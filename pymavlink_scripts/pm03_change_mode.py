from pymavlink import mavutil
import sys

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()

# 変更後のモード
#mode = "RTL"
mode = "AUTO"


# モードが有効かをチェック
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# モードIDを取得
mode_id = master.mode_mapping()[mode]

# モード変更リクエストを送信
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)

# master.set_mode(mode_id) # モードID指定でのモード変更
# master.set_mode_loiter() # LOITER専用のモード変更

# モード変更の確認を行う
while True:
    if master.flightmode == mode:
        break
    master.recv_msg()

print("変更後モード:", master.flightmode)

# モード変更は便利関数を使ってもOK
# master.set_mode(mode_id) # モードID指定でのモード変更
# master.set_mode_loiter() # LOITER専用のモード変更

# モード変更は非推奨だが下記でもOK
# master.mav.set_mode_send(
#     master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

# モード変更の確認は下記でもOK
# while True:
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
#     ack_msg = ack_msg.to_dict()
#     if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
#         continue
#     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
#     break
