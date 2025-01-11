from pymavlink import mavutil

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:192.168.3.39:5763", source_system=1, source_component=90)
master.wait_heartbeat()

# メッセージ受信
recieved_msg = master.recv_match(type='HEARTBEAT', blocking=True)
print(recieved_msg)

# メッセージ直接送信
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    0, 0, 0)

# メッセージ作成して送信
to_send_msg = master.mav.heartbeat_encode(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    0, 0, 0)
print(to_send_msg)
master.mav.send(to_send_msg)

# Mavlink Inspectorを開いてシステムID、コンポーネントIDを確認する
# HEARTBEATメッセージ定義は下記を参照
# https://mavlink.io/en/messages/common.html#HEARTBEAT