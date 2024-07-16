# pymavlink版のtake-offコードの写経 Kei.Kabutomori
from pymavlink import mavutil
import time

master: mavutil.mavfile = mavutil.mavlink_connection("127.0.0.1:14551",  source_system=1, source_component=90)
master.wait_heartbeat()
print("接続完了")

mode = 'GUIDED'
master.set_mode_apm(master.mode_mapping()[mode])

while True:
    if master.flightmode == mode:
        break
    master.recv_msg()
print("モード変更完了")

master.arducopter_arm()
print("アーム開始")
master.motors_armed_wait()
print("アーム完了")

target_altitude = 10 # 3m→10mに変更

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, target_altitude)

# GLOBAL_POSITION_INT(33)を10Hzで受信
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0)

# 元のコードでコメントアウトされていたけど、未使用でOK？
#master.mav.global_position_int_send(
#   0, 0, 0, 0, 0, 0, 0, 0, 0)

# 目標高度への到達チェック
while True:
    recieved_msg = master.recv_match(
        type='GLOBAL_POSITION_INT', blocking=True)
    current_altitude = recieved_msg.relative_alt / 1000
    print("現在の高度: {}".format(current_altitude))

    if current_altitude >= target_altitude * 0.95:
        print("目標高度に到達しました")
        break

    time.sleep(0.1)

# 終了処理を追加(Trial)
mode = 'RTL'
master.set_mode_apm(master.mode_mapping()[mode])
time.sleep(0.1)

while True:
    if master.flightmode == mode:
        break
    master.recv_msg()

print("プログラムが終了したので、着陸します。")
time.sleep(0.1)

master.close()
