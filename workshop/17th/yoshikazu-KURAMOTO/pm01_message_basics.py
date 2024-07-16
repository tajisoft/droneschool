from pymavlink import mavutil

# 機体への接続
# mavlinkrouter - raspberry pi - USB FTDI - FC
master: mavutil.mavfile = mavutil.mavlink_connection(
    #"/dev/ttyUSB0", baudrate=115200, source_system=1, source_component=90
    #'udpin:localhost:14551', source_system=1, source_component=90
    "127.0.0.1:14551", source_system=1, source_component=90
)

# メッセージ受信
received_msg = master.recv_match(type='HEARTBEAT', blocking=True)
print(received_msg)

# メッセージ直接送信
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    0, 0, 0
)

# メッセージ作成して送信
to_send_msg = master.mav.heartbeat_encode(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    0, 0, 0
)
print(to_send_msg)
master.mav.send(to_send_msg)

# Mavlink Inspectorを開いてシステムID、コンポーネントIDを確認する
# HEARTBEATメッセージ定義は下記を参照
# https://mavlink.io/en/messages/common.html#HEARTBEAT
#
#実行結果　Mavlink Inspectorに以下の表示
# Vehicle 1
#    |- + Comp 1 MAV_COMP_ID_AUTOPILOT1
#    |- + Comp 90 MAV_COMP_ID_USER66
