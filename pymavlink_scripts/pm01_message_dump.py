from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:192.168.3.36:5762", source_system=1, source_component=90)
master.wait_heartbeat()

# 全メッセージを10Hzで受信
# master.mav.request_data_stream_send(
#     master.target_system, master.target_component,
#     0, 10, 1)

# GLOBAL_POSITION_INT(33)メッセージを10Hzで受信
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0)

while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.01)
