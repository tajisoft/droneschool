import threading
import time
import signal
import sys
from flight_experience_ota import (
    connect_to_mavlink, change_mode_and_confirm, arm_drone, takeoff_to_altitude
)
from mission import convert_dict_to_mission_items, generate_do_jump_item, upload_mission

class QuadplanePatrolThread(threading.Thread):
    def __init__(self, connection_string):
        super().__init__()
        self.connection_string = connection_string
        self._stop_event = threading.Event()

    def run(self):
        patrol_route = {
            "1": {"lat": 35.88233960, "lon": 140.34802440, "alt": 100.0},
            "2": {"lat": 35.87343770, "lon": 140.33523560, "alt": 100.0},
            "3": {"lat": 35.86676060, "lon": 140.32356260, "alt": 100.0},
            "4": {"lat": 35.86443050, "lon": 140.31472210, "alt": 100.0},
            "5": {"lat": 35.86450000, "lon": 140.30643940, "alt": 100.0},
            "6": {"lat": 35.86700410, "lon": 140.29807090, "alt": 100.0},
            "7": {"lat": 35.87089910, "lon": 140.29008870, "alt": 100.0},
            "8": {"lat": 35.86700410, "lon": 140.29807090, "alt": 100.0},
            "9": {"lat": 35.86450000, "lon": 140.30643940, "alt": 100.0},
            "10": {"lat": 35.86443050, "lon": 140.31472210, "alt": 100.0},
            "11": {"lat": 35.86676060, "lon": 140.32356260, "alt": 100.0},
            "12": {"lat": 35.87343770, "lon": 140.33523560, "alt": 100.0},
            "13": {"lat": 35.88233960, "lon": 140.34802440, "alt": 100.0}
        }

        master = connect_to_mavlink(self.connection_string)
        change_mode_and_confirm(master, "GUIDED")
        arm_drone(master)
        takeoff_to_altitude(master, target_alt=30.0)

        try:
            msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            if msg is None:
                raise RuntimeError("Failed to get current position for dummy WP")
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0

            mission_items = convert_dict_to_mission_items(
                patrol_route,
                master.target_system,
                master.target_component,
                current_lat,
                current_lon,
                current_alt
            )
            do_jump = generate_do_jump_item(
                master.target_system,
                master.target_component,
                seq=len(mission_items),
                jump_to_seq=1
            )
            mission_items.append(do_jump)

            upload_mission(master, mission_items)
            master.mav.mission_set_current_send(master.target_system, master.target_component, 1)
            change_mode_and_confirm(master, "AUTO")

            while not self._stop_event.is_set():
                print("\r巡回中... (CTRL+Cで停止)                ", end="", flush=True)
                time.sleep(1)

        except Exception as e:
            print(f"\nスレッド例外: {e}")
        finally:
            print("\nQuadplaneスレッド停止、RTLへ")
            change_mode_and_confirm(master, "QRTL")

    def stop(self):
        self._stop_event.set()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", type=str, default="tcp:172.26.176.1:5763")
    args = parser.parse_args()

    patrol_thread = QuadplanePatrolThread(args.connect)

    def signal_handler(sig, frame):
        print("\nCtrl+C検知、スレッド停止要求送信中...")
        patrol_thread.stop()
        patrol_thread.join()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    patrol_thread.start()
    patrol_thread.join()
