import signal
import sys
from threading import Event
from rover_transport import RoverMissionThread
from flight_experience_ota import change_mode_and_confirm

# --- イベント定義 ---
event_rover2_start = Event()

debug_prefix = lambda name: f"[{name}]"

# --- Rover1: ボートの役割 + イベント送信 ---
class RoverWithTrigger(RoverMissionThread):
    def __init__(self, *args, trigger_event=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.trigger_event = trigger_event

    def hold_and_wait(self, description):
        print(f"{debug_prefix(self.name)} {description}完了 → HOLD")
        change_mode_and_confirm(self.master, "HOLD")
        if description == "Forward" and self.trigger_event:
            print(f"{debug_prefix(self.name)} Rover2 起動イベント送信")
            self.trigger_event.set()
        print(f"{debug_prefix(self.name)} 続行待機中...")
        self._event.wait()
        self._event.clear()
        print(f"{debug_prefix(self.name)} AUTO再開")
        change_mode_and_confirm(self.master, "AUTO")


# --- Rover2: 荷物運搬役 + イベント受信して開始 ---
class RoverWaitStart(RoverMissionThread):
    def __init__(self, *args, wait_event=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.wait_event = wait_event

    def run(self):
        print(f"{debug_prefix(self.name)} イベント待機中...")
        self.wait_event.wait()
        self.wait_event.clear()
        print(f"{debug_prefix(self.name)} スタート！")
        super().run()


# --- ルート定義 ---
boat_forward = {
    "1": {"lat": 35.87781700, "lon": 140.33848640, "alt": 0.0},
    "2": {"lat": 35.88069660, "lon": 140.34805120, "alt": 0.0},
    "3": {"lat": 35.87976800, "lon": 140.34849500, "alt": 0.0}
}
boat_return = {
    "1": {"lat": 35.88014900, "lon": 140.34816920, "alt": 0.0},
    "2": {"lat": 35.87779310, "lon": 140.33843280, "alt": 0.0},
    "3": {"lat": 35.87827500, "lon": 140.33806900, "alt": 0.0}
}

rover_forward = {
    "1": {"lat": 35.8797056, "lon": 140.3484374, "alt": 0.0},
    "2": {"lat": 35.8795318, "lon": 140.3485420, "alt": 0.0},
    "3": {"lat": 35.8793351, "lon": 140.3485246, "alt": 0.0},
    "4": {"lat": 35.8790167, "lon": 140.3482577, "alt": 0.0},
    "5": {"lat": 35.8789156, "lon": 140.3482403, "alt": 0.0},
    "6": {"lat": 35.8788341, "lon": 140.3483543, "alt": 0.0},
    "7": {"lat": 35.8786331, "lon": 140.3480619, "alt": 0.0},
    "8": {"lat": 35.8779300, "lon": 140.3489175, "alt": 0.0},
    "9": {"lat": 35.8773574, "lon": 140.3484039, "alt": 0.0},
    "10": {"lat": 35.876991, "lon": 140.348026, "alt": 0.0}
}
rover_return = {
    "1": {"lat": 35.8773574, "lon": 140.3484039, "alt": 0.0},
    "2": {"lat": 35.8779300, "lon": 140.3489175, "alt": 0.0},
    "3": {"lat": 35.8786331, "lon": 140.3480619, "alt": 0.0},
    "4": {"lat": 35.8788341, "lon": 140.3483543, "alt": 0.0},
    "5": {"lat": 35.8789156, "lon": 140.3482403, "alt": 0.0},
    "6": {"lat": 35.8790167, "lon": 140.3482577, "alt": 0.0},
    "7": {"lat": 35.8793351, "lon": 140.3485246, "alt": 0.0},
    "8": {"lat": 35.8795318, "lon": 140.3485420, "alt": 0.0},
    "9": {"lat": 35.8797056, "lon": 140.3484374, "alt": 0.0},
    "10": {"lat": 35.879768, "lon": 140.348495, "alt": 0.0}
}

# --- スレッド生成 ---
rover1 = RoverWithTrigger("tcp:172.26.176.1:5773", boat_forward, boat_return,
                          name="Rover1(Boat)", trigger_event=event_rover2_start)

rover2 = RoverWaitStart("tcp:172.26.176.1:5803", rover_forward, rover_return,
                        name="Rover2(Cargo)", wait_event=event_rover2_start)

# --- シグナルハンドラ ---
def signal_handler(sig, frame):
    print("[Ctrl+C] 停止中...")
    rover1.stop()
    rover2.stop()
    rover1.join()
    rover2.join()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# --- 実行 ---
print("== Rover1, Rover2 同時起動 (連携制御) ==")
rover1.start()
rover2.start()
rover1.join()
rover2.join()
