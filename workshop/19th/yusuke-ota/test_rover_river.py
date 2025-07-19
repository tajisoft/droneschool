import threading
import time
import signal
import sys
from rover_transport import RoverMissionThread

def main():
    forward_route = {
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

    return_route = {
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


    rover = RoverMissionThread("tcp:172.26.176.1:5813", forward_route, return_route)

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