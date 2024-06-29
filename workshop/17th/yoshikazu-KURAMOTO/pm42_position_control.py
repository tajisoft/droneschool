from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    device="127.0.0.1:14551", source_system=1, source_component=90
)

# MAVLinkメッセージ作成
msg = master.mav.set_position_target_local_ned_encode(
    0,
    0, 0,
    mavutil.mavlink.MAVFRAME_LOCAL_NED,
    0b0000111111000111,
    0, 0, 0,
    2, -2, 1,
    0, 0, 0,
    0, 0
)

# MAVLinkメッセージ送信
for x in range(0, 20):
    master.mav.send(msg)
    time.sleep(0.1)