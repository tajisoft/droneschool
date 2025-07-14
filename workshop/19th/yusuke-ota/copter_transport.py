# copter_transport.py
import threading
import time
import signal
import sys
from pymavlink import mavutil
from flight_experience_ota import connect_to_mavlink, change_mode_and_confirm, arm_drone, takeoff_to_altitude
from mission import convert_dict_to_mission_items, upload_mission

class CopterTransportThread(threading.Thread):
    def __init__(self, connection_string):
        super().__init__()
        self.connection_string = connection_string
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        # 接続
        master = connect_to_mavlink(self.connection_string)

        # 現在位置取得（ダミーWP用）
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None:
            raise RuntimeError("Failed to get current position")
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000.0

        # 往復ルート作成
        forward = {"1": {"lat": 35.867003, "lon": 140.305987, "alt": 30.0}}  # 着陸地点
        return_ = {"1": {"lat": 35.878275, "lon": 140.338069, "alt": 30.0}}  # 出発地点

        items = []

        # 離陸地点から移動（ダミー）
        forward_items = convert_dict_to_mission_items(
            forward,
            target_sys=master.target_system,
            target_comp=master.target_component,
            current_lat=current_lat,
            current_lon=current_lon,
            current_alt=current_alt,
            include_dummy=True
        )
        forward_items[-1].command = mavutil.mavlink.MAV_CMD_NAV_LAND  # 最後は着陸
        items.extend(forward_items)

        # ミッションアップロード
        upload_mission(master, items)
        master.mav.mission_set_current_send(master.target_system, master.target_component, 1)
        try:
            change_mode_and_confirm(master, "GUIDED")
            arm_drone(master)
            takeoff_to_altitude(master, target_alt=30.0)
            while not self._stop_event.is_set():
                print("\r運搬中...", end="", flush=True)
                time.sleep(1)
        except Exception as e:
            print(f"\n例外: {e}")


        # フライト準備




        print("Copter運搬開始（CTRL+Cで停止）")
        try:
            while not self._stop_event.is_set():
                print("\r運搬中...", end="", flush=True)
                time.sleep(1)
        except Exception as e:
            print(f"\n例外: {e}")
        finally:
            print("\nRTLモードへ移行")
            change_mode_and_confirm(master, "RTL")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", type=str, default="tcp:127.0.0.1:5763")
    args = parser.parse_args()

    copter_thread = CopterTransportThread(args.connect)

    def signal_handler(sig, frame):
        print("\n[Ctrl+C] スレッド停止中...")
        copter_thread.stop()
        copter_thread.join()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    copter_thread.start()
    copter_thread.join()
