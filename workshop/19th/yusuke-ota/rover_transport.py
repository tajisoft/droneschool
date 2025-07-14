import threading
import time
import signal
import sys
from flight_experience_ota import (
    connect_to_mavlink, change_mode_and_confirm, arm_drone
)
from mission import (
    convert_dict_to_mission_items, generate_do_jump_item, upload_mission, generate_loiter_time_item
)
from pymavlink import mavutil


class RoverMissionThread(threading.Thread):
    def __init__(self, connection_string, forward_route, return_route, name="Rover"):
        super().__init__()
        self.connection_string = connection_string
        self.forward_route = forward_route
        self.return_route = return_route
        self.name = name
        self._event = threading.Event()
        self._stop_event = threading.Event()
        self.master = None
        self.forward_callback = None
        self.return_callback = None

    def set_event(self):
        self._event.set()

    def stop(self):
        self._stop_event.set()
        self._event.set()

    def set_forward_callback(self, func):
        self.forward_callback = func

    def set_return_callback(self, func):
        self.return_callback = func

    def hold_and_wait(self, description, callback=None):
        print(f"[{self.name}] {description}完了 → HOLD")
        change_mode_and_confirm(self.master, "HOLD")
        if callback:
            print(f"[{self.name}] コールバック実行: {callback.__name__}")
            callback()
        print(f"[{self.name}] 次のイベント待機中...")
        self._event.wait()
        self._event.clear()
        print(f"[{self.name}] 再開 → AUTO")
        change_mode_and_confirm(self.master, "AUTO")

    def wait_for_mission_seq(self, target_seq):
        while not self._stop_event.is_set():
            msg = self.master.recv_match(type="MISSION_CURRENT", blocking=True, timeout=1)
            if msg and msg.seq == target_seq:
                print(f"[{self.name}] 到達確認: seq={target_seq}")
                return True
            time.sleep(0.5)
        return False

    def append_loiter_at_end(self, mission_items, wp, seq):
        mission_items.append(generate_loiter_time_item(
            self.master.target_system, self.master.target_component,
            seq=seq, lat=wp["lat"], lon=wp["lon"], alt=wp["alt"], loiter_time=1.0
        ))
        return len(mission_items) - 1

    def run(self):
        try:
            self.master = connect_to_mavlink(self.connection_string)
            change_mode_and_confirm(self.master, "GUIDED")
            arm_drone(self.master)

            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            if msg is None:
                raise RuntimeError("位置情報が取得できません")
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0

            mission_items = []

            # forward追加
            forward_items = convert_dict_to_mission_items(
                self.forward_route, self.master.target_system, self.master.target_component,
                current_lat, current_lon, current_alt, start_seq=len(mission_items),include_dummy=True
            )
            mission_items.extend(forward_items)
            forward_loiter_seq = self.append_loiter_at_end(mission_items, list(self.forward_route.values())[-1], len(mission_items))

            # return追加
            return_items = convert_dict_to_mission_items(
                self.return_route, self.master.target_system, self.master.target_component,
                current_lat, current_lon, current_alt, start_seq=len(mission_items)
            )
            mission_items.extend(return_items)
            return_loiter_seq = self.append_loiter_at_end(mission_items, list(self.return_route.values())[-1], len(mission_items))

            # DO_JUMP
            mission_items.append(generate_do_jump_item(
                self.master.target_system, self.master.target_component,
                seq=len(mission_items), jump_to_seq=1
            ))

            upload_mission(self.master, mission_items)
            self.master.mav.mission_set_current_send(self.master.target_system, self.master.target_component, 1)

            print(f"[{self.name}] ミッションループ開始")

            while not self._stop_event.is_set():
                self.hold_and_wait("preparation", callback=self.forward_callback)

                self.wait_for_mission_seq(forward_loiter_seq)
                self.hold_and_wait("Forward", callback=self.forward_callback)

                self.wait_for_mission_seq(return_loiter_seq)

        except Exception as e:
            print(f"[{self.name}] 例外: {e}")
        finally:
            if self.master:
                print(f"[{self.name}] 終了処理 → RTL")
                change_mode_and_confirm(self.master, "RTL")

if __name__ == "__main__":
    forward_route = {
        "1": {"lat": 35.87781700, "lon": 140.33848640, "alt": 0.0},
        "2": {"lat": 35.88069660, "lon": 140.34805120, "alt": 0.0},
        "3": {"lat": 35.87976800, "lon": 140.34849500, "alt": 0.0}
    }

    return_route = {
        "1": {"lat": 35.88014900, "lon": 140.34816920, "alt": 0.0},
        "2": {"lat": 35.87779310, "lon": 140.33843280, "alt": 0.0},
        "3": {"lat": 35.87827500, "lon": 140.33806900, "alt": 0.0}
    }

    rover = RoverMissionThread("tcp:172.26.176.1:5773", forward_route, return_route)

    def signal_handler(sig, frame):
        print("\n[Ctrl+C] 停止要求中...")
        rover.stop()
        rover.join()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    rover.start()

    try:
        while True:
            input("Enterキーでローバー再開: ")
            rover.set_event()
    except KeyboardInterrupt:
        signal_handler(None, None)
        

