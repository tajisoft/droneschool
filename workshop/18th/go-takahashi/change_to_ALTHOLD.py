from pymavlink import mavutil
import sys

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
device="tcp:10.40.228.251:5762", source_system=1, source_component=90)
master.wait_heartbeat()

# 変更後のモード
mode = "ALT_HOLD"

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

while True:
    if master.flightmode == mode:
        break
    master.recv_msg()

print( master.flightmode,"モードに変更しました")