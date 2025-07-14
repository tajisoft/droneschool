import threading
import time
import signal
import sys
from rover_transport import RoverMissionThread

def main():
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

if __name__ == "__main__":
    main()