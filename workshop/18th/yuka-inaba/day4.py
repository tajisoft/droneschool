### エクスペリエンス用 ###

from pymavlink import mavutil
import time

# 接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "udp:127.0.0.1:14551", baud=57600,  source_system=1, source_component=90)
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


## 離陸 (現在高度から +2m) ##
target_altitude = 2
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, target_altitude)

# メッセージレート変更: GLOBAL_POSITION_INT(33)を10Hzで受信
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0)

# GLOBAL_POSITION_INT メッセージを要求
master.mav.global_position_int_send(0, 0, 0, 0, 0, 0, 0, 0, 0)

# 目標高度への到達を確認
while True:
    # GLOBAL_POSITION_INT から相対高度を取得
    recieved_msg = master.recv_match(
        type='GLOBAL_POSITION_INT', blocking=True)
    current_altitude = recieved_msg.relative_alt / 1000

    print("高度: {}cm".format(current_altitude * 100))

    if current_altitude >= target_altitude * 0.95:
        print("目標高度に到達")
        break

time.sleep(1)


## 0.7m前進 ##
# メッセージを送信
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # 機体の位置を原点とする
    0b0000111111111000,
    0.7, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,)

# 移動が完了するまで待機
time.sleep(5)


## 10秒降下 ##
print("降下開始")
# MAVLINKメッセージを作成：0.1m/sで降下
msg = master.mav.set_position_target_local_ned_encode(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # 機体の位置を原点とする
    0b0000111111000111,
    0, 0, 0,  # x, y, z位置 (使用しない)
    0, 0, 0.1,  # x, y, z速度(down) (m/s)
    0, 0, 0,  # x, y, z加速度 (使用しない)
    0, 0)  # yaw, yaw_rate (使用しない))

#メッセージ送信
i = 0
for x in range(1000):
    master.mav.send(msg)

    # 圧力センサの測定値が閾値を超えたら抜ける（iは練習用）
    i += 1

    if abs(i - 100) < 0.001:  # iが100にほぼ等しい場合
      print("信号を受信したため降下を停止")
      break

    time.sleep(0.1)


## 上昇 ##
print("上昇開始")
# MAVLINKメッセージを作成：2mまで上昇
master.mav.set_position_target_local_ned_send(
    0,                              # 起動からの時間
    master.target_system,           # ターゲットシステムID
    master.target_component,        # ターゲットコンポーネントID
    mavutil.mavlink.MAV_FRAME_BODY_NED,  # 地面基準のNED座標系
    0b0000111111111000,             # 位置制御のみ有効
    0, 0, -1,                       # zは-1m（上昇）
    0, 0, 0,                        # 速度（無効）
    0, 0, 0,                        # 加速度（無効）
    0, 0                            # yaw, yaw_rate（無効）
)

# 移動が完了するまで待機
time.sleep(5)


## 0.7m後進 ##
# メッセージを送信
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # 機体の位置を原点とする
    0b0000111111111000,
    -0.7, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,)

# 移動が完了するまで待機
time.sleep(5)

## 着陸 ##
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(10)
print("着陸")

# 切断
master.close()