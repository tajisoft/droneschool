import threading
import time
import signal
import sys
from itertools import cycle
from flight_experience_ota import (
    connect_to_mavlink, change_mode_and_confirm, arm_drone,
    send_guided_waypoints_from_dict, set_and_verify_param
)
from pymavlink import mavutil

class RoverTransportThread(threading.Thread):
    def __init__(self, connection_string, route_list, name="Rover", on_arrival=None):
        super().__init__()
        self.connection_string = connection_string
        self.route_iterator = cycle(route_list)
        self.name = name
        self.on_arrival = on_arrival
        self._event = threading.Event()
        self._stop_event = threading.Event()
        self.master = None

    def set_event(self):
        self._event.set()

    def stop(self):
        self._stop_event.set()
        self._event.set()  # unblock wait if needed

    def run(self):
        try:
            print(f"[{self.name}] Connecting to MAVLink...")
            self.master = connect_to_mavlink(self.connection_string)

            print(f"[{self.name}] Setting speed to 20 m/s...")
            set_and_verify_param(
                self.master,
                "CRUISE_SPEED",
                20,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            set_and_verify_param(
                self.master,
                "WP_SPEED",
                20,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )

            if not change_mode_and_confirm(self.master, "GUIDED"):
                print(f"[{self.name}] Failed to enter GUIDED mode")
                return
            if not arm_drone(self.master):
                print(f"[{self.name}] Failed to arm")
                return

            for route in self.route_iterator:
                print(f"[{self.name}] Waiting for event...")
                self._event.wait()
                if self._stop_event.is_set():
                    break
                self._event.clear()

                print(f"[{self.name}] Starting route...")
                success = send_guided_waypoints_from_dict(
                    self.master, route, threshold_m=10.0, timeout=180.0
                )
                if success:
                    print(f"[{self.name}] Reached destination.")
                    if self.on_arrival:
                        self.on_arrival(self.name)
                else:
                    print(f"[{self.name}] Failed to reach destination.")

        except Exception as e:
            print(f"[{self.name}] Exception: {e}")

        finally:
            print(f"[{self.name}] Stopping and changing to RTL")
            try:
                if self.master:
                    change_mode_and_confirm(self.master, "RTL")
            except Exception as e:
                print(f"[{self.name}] Exception during RTL: {e}")


if __name__ == "__main__":
    # テストルート（往路 + 復路）
    forward_route = {
        "1": {"lat": 35.87781700, "lon": 140.33848640, "alt": 0.0},
        "2": {"lat": 35.88069660, "lon": 140.34805120, "alt": 0.0},
        "3": {"lat": 35.87976800, "lon": 140.34849500, "alt": 0.0}  # 目標地
    }

    return_route = {
        "1": {"lat": 35.88014900, "lon": 140.34816920, "alt": 0.0},
        "2": {"lat": 35.87779310, "lon": 140.33843280, "alt": 0.0},
        "3": {"lat": 35.87827500, "lon": 140.33806900, "alt": 0.0}  # 目標地
    }

    routes = [forward_route, return_route]

    def on_arrival(name):
        print(f"[{name}] 到着しました！次の入力を待っています...")

    rover = RoverTransportThread("tcp:172.26.176.1:5773", routes, name="TestRover", on_arrival=on_arrival)

    def signal_handler(sig, frame):
        print("\nCtrl+C detected. Stopping rover...")
        rover.stop()
        rover.join()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    rover.start()

    try:
        while True:
            input("Enterキーでローバー出発: ")
            rover.set_event()
    except KeyboardInterrupt:
        signal_handler(None, None)