from pymavlink import mavutil

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()

master.arducopter_arm()
master.motors_armed_wait()
print("ARMED")

time.sleep(5)

master.arducopter_disarm()
master.motors_disarmed_wait()
print("DISARMED")
