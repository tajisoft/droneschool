from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90
)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

#ARM
#master.arducopter_arm()
# または:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

#ARM終了まで待つ master.motors_armed()を使うとマニュアルでチェックできる
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
#master.motors_armed()
print("ARMED")

time.sleep(5)

#DISARM
#master.arducopter_disarm()
#　または:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)

#DISARMとなるまで待つ
master.motors_disarmed_wait()
print("DISARMED")