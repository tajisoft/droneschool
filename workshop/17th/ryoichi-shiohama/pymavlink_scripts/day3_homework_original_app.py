from pymavlink import mavutil
import time

# 機体（シミュレータ）への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
  "tcp:10.0.2.135:5762",
  source_system=1,
  source_component=90
)
master.wait_heartbeat()
print("接続完了")

# GUIDEDモードに変更
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
target_altitude = 3

# 離陸
master.mav.command_long_send(
  master.target_system,
  master.target_component,
  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
  0, 0, 0, 0, 0, 0, 0,
  target_altitude
)

# TODO: メッセージレート変更か？検討する
master.mav.command_long_send(
  master.target_system,
  master.target_component,
  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
  0, 33, 100000, 0, 0, 0, 0, 0
)

# 目標高度への到達を確認
while True:
  # GLOBAL_POSITION_INT から相対高度を取得
  recieved_msg = master.recv_match(
    type='GLOBAL_POSITION_INT',
    blocking=True
  )
  current_altitude = recieved_msg.relative_alt / 1000

  print(f"高度:{current_altitude}")

  if current_altitude >= target_altitude * 0.95:
    print("目標高度に到達")
    break

  time.sleep(0.1)

# TODO:目標高度到達後、任意の軌道を描く（まずは三角など、次に努力目標で星？）

# TODO: RTLできたらしたい

# 切断
master.close()
