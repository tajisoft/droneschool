from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("udpin:127.0.0.1:14551")

master.wait_heartbeat()

while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(1.0)
