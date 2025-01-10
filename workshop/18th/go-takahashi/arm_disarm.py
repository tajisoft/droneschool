from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:10.40.228.251:5762", source_system=1, source_component=90)
master.wait_heartbeat()

master.arducopter_arm()
master.motors_armed_wait()
print("ARMED")

time.sleep(10)

master.arducopter_disarm()
master.motors_disarmed_wait()
print("DISARMED")
