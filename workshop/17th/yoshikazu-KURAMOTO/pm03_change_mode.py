from pymavlink import mavutil
import sys

# 機体への接続
master:mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90
)
# コマンドを送る前にheartbeatを確認
master.wait_heartbeat()


# モード選択
# ArduSub4.6.0で使用できるモード
# STABILIZE      :   0
# ACRO           :   1
# ALT_HOLD       :   2 (need alt estimate)
# AUTO           :   3 (need alt estimate)
# GUIDED         :   4 (need alt estimate)
# CIRCLE         :   7 (need alt estimate)
# SURFACE        :   9 (need alt estimate)
# POSHOLD        :  16 (need alt estimate)
# MANUAL         :  19 
# MOTOR_DETECT   :  20
# SURFTRAK       :  21 (need alt estimate)


mode = "STABILIZE"

# Check if mode is available
# ArduSubのモードについてはSTABILIZEが正常に動く
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]


# 設定するモードとID番号表示
print("mode: {}".format(mode))
print("modeid:{}".format(mode_id))

# モード変更リクエストを送信
# master.mav.command_long_send(
#     master.target_system, master.target_component,
#     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#     #mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0
#)

# master.set_mode(mode_id) # モードID指定でのモード変更
# master.set_mode_loiter() # LOITER専用モード変更

# モード変更の確認を行う
# ArduSubではFlightmodeの番号が異なるようでループを抜け出せない
# while True:
#     if master.flightmode == mode:
#         break
#     master.recv_msg()

# モード変更は便利関数を使っても可
#  master.set_mode(mode_id) # モードID指定でのモード変更
#  master.set_mode_loiter() # LOITER専用モード変更

# モード変更は非推奨だが下記でも可
# master.mav.set_mode_send(
#     master.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id
# )

# モードの変更確認は下記でも可
while True:
    ack_msg = master.recv_match(type='COMMAND_ACK',blocking=True)
    ack_msg = ack_msg.to_dict()
    if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        continue
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

# ArduSubではモード番号が異なるのでUNKNOWN表示
print("changed mode:", master.flightmode)

