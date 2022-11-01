import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("udpin:127.0.0.1:14551")

master.wait_heartbeat()

# Arm
master.arducopter_arm()
master.motors_armed_wait()
print("ARMED")

time.sleep(10)

# Disarm
master.arducopter_disarm()
master.motors_disarmed_wait()
print("DISARMED")
