from pymavlink import mavutil
import time
import math

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()

print("接続完了")


# GUIDEDにモード変更
mode = 'GUIDED'
master.set_mode_apm(master.mode_mapping()[mode])

# モード変更を確認
while True:
    if master.flightmode == mode:
        break
    master.recv_msg()
print("モード変更完了")

# アーム
master.arducopter_arm()
master.motors_armed_wait()
print("アーム完了")

# 目標高度
target_altitude = 10

# 離陸
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, target_altitude)

# メッセージレート変更: GLOBAL_POSITION_INT(33)を10Hzで受信
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0)


# 目標高度への到達を確認
while True:
    # GLOBAL_POSITION_INT から相対高度を取得
    recieved_msg = master.recv_match(
        type='GLOBAL_POSITION_INT', blocking=True)
    current_altitude = recieved_msg.relative_alt / 1000

    print("高度: {}".format(current_altitude))

    if current_altitude >= target_altitude * 0.95:
        print("目標高度に到達")
        break

    time.sleep(0.1)

## 30m前進 ##
# メッセージを送信
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # 機体の位置を原点とする
    0b0000111111111000,
    10, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,)

# 移動が完了するまで待機
time.sleep(5)
print("10m北方向に移動しました")



## 10m東方向に移動 ##
# メッセージを送信
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # 機体の位置を原点とする
    0b0000111111111000,
    0, 10, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,)
# 移動が完了するまで待機
time.sleep(5)
print("10m東方向に移動しました")




## 着陸 ##
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(10)
print("着陸")

# 切断
master.close()



