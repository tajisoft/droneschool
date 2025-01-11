from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection("tcp:192.168.1.57:5762", source_system=1, source_component=90)
master.wait_heartbeat()

# GLOBAL_POSITION_INT(33)メッセージを10Hzで受信
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 33, 100000, 0, 0, 0, 0, 0)

while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass

    time.sleep(1.0)
