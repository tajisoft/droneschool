from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
<<<<<<< HEAD
    # "127.0.0.1:14551",
    "tcp:10.0.2.135:5762",
    source_system=2, source_component=91)
=======
    "127.0.0.1:14551", source_system=1, source_component=90)
>>>>>>> upstream/master
master.wait_heartbeat()

# ターゲットシステムID、コンポーネントIDを表示
print(f"target_system: {master.target_system}, target_component: {master.target_component}")

# HEARTBEATメッセージを1秒おきに送信
while True:
    time.sleep(1)

    # メッセージ直接送信
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0, 0, 0)

    # メッセージ作成して送信
    # to_send_msg = master.mav.heartbeat_encode(
    #     mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    #     mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    #     0, 0, 0 )
    # master.mav.send(to_send_msg)
