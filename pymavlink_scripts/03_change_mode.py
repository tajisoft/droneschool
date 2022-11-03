import sys
from pymavlink import mavutil

master = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
# master = mavutil.mavlink_connection("udpin:127.0.0.1:14551")

master.wait_heartbeat()

# 変更後モード
mode = 'STABILIZE'

# モードが有効かをチェック
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# モードIDを取得
mode_id = master.mode_mapping()[mode]

# モード変更リクエストを送信
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

# モード変更の正常終了をチェック
while True:
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()
    
    if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        continue

    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break