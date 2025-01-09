from pymavlink import mavutil
import sys
import time

# 機体への接続
#master: mavutil.mavfile = mavutil.mavlink_connection(
#    "127.0.0.1:14551", source_system=1, source_component=90)
master: mavutil.mavfile = mavutil.mavlink_connection(
    "/dev/serial0", baud=115200, source_system=1, source_component=90)
master.wait_heartbeat()

# 変更後のモード
#mode = "RTL"
mode = "LOITER"

# モードが有効かをチェック
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# モードIDを取得
mode_id = master.mode_mapping()[mode]

# モード変更リクエストを送信
# ==>これはやらずにプロポで変更して取得できるかの実験に変更
#master.mav.command_long_send(
#   master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)

# master.set_mode(mode_id) # モードID指定でのモード変更
# master.set_mode_loiter() # LOITER専用のモード変更

# モード変更の確認を行う
# 元のサンプルだとrecv_msg()を使っているがモード設定しない時のモード取得に失敗する
#   ==> recv_match()だとモード変化を取得できることを実機：ラズベリーパイ接続で確認した
#       但し、sleep(1)だと変化検出できないがsleep(0.1)だと検出できる
#       ※sleep値を変更して実験したところ、0.1 or 0.2より長い時間のsleepだとまともに動かない
while True:
    if master.flightmode == mode:
        break
    try:
      #master.recv_msg()
      msg = master.recv_match(type='SYS_STATUS',blocking=True)
      print(msg.mode)
    except:
      pass
    print("現在モード:", master.flightmode)
    time.sleep(0.2)

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
