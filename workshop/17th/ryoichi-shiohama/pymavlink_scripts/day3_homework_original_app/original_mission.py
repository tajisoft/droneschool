from pymavlink import mavutil
import time
from math import radians, sin, cos, sqrt, atan2
from mission_utils import create_triangle_mission

# 自動航行の処理を実行する（三角形ウェイポイント設定 > Autoモード）
def do_triangle_mission(master, target_altitude):
  # ミッションの作成とアップロード
  mission = create_triangle_mission(master, target_altitude)

  # ミッションモードに変更
  mission_mode = 'AUTO'
  master.set_mode_apm(master.mode_mapping()[mission_mode])
  print("AUTOモードに変更しました")

  # ミッション開始を確認
  while True:
      master.recv_msg()
      if master.flightmode == mission_mode:
          print("ミッションモードに変更完了")
          break

  # ミッション完了を待機
  while True:
      msg = master.recv_match(type=['MISSION_ITEM_REACHED', 'MISSION_CURRENT'], blocking=True)
      if msg.get_type() == 'MISSION_ITEM_REACHED' and msg.seq == len(mission) - 1:
          print("ミッション完了")
          break
      time.sleep(0.1)


# ホームロケーションとの距離を計算する
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # 地球の半径（メートル）
    dlat = radians(lat2 - lat1) # lat distance
    dlon = radians(lon1 - lon2) # lon distance

    # 球面上の2点間の距離を計算（ハバーサインの公式）
    a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) * sin(dlon / 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance


# 機体（シミュレータ）への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
  "tcp:10.0.2.134:5762",
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

# 一度だけHOME_POSITIONメッセージをリクエスト
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0, 242, 0, 0, 0, 0, 0, 0
)

# ホームロケーションを取得
print("ホームロケーションを取得")
home_latitude = 0
home_longitude = 0
home_altitude = 0
while True:
  home_location_msg = master.recv_match(type='HOME_POSITION', blocking=True)
  if home_location_msg is not None:
    # ホームロケーション情報を設定
    home_latitude = home_location_msg.latitude / 1.0e7
    home_longitude = home_location_msg.longitude / 1.0e7
    home_altitude = home_location_msg.altitude / 1000.0
    print(f"ホームロケーション: lat={home_latitude},lon={home_longitude},alt={home_altitude}")
    break

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

# メッセージレート変更: GLOBAL_POSITION_INT(33)を10Hzで受信
# 離陸判定のほか、create_missionでもGLOBAL_POSITION_INTを受信するのに必要
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


# 目標高度到達後、三角形の軌道を描く
do_triangle_mission(master, target_altitude)


# RTLモードに変更
time.sleep(1)

rtl_mode = 'RTL'
master.set_mode_apm(master.mode_mapping()[rtl_mode])

while True:
  master.recv_msg()
  if master.flightmode == rtl_mode:
    print("RTLモードに変更完了")
    break

# ホームロケーションに戻るのを待つ
while True:
  received_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
  current_lat = received_msg.lat / 1.0e7
  current_lon = received_msg.lon / 1.0e7

  # ホームロケーションとの距離を計算する
  distance_to_home = calculate_distance(current_lat, current_lon, home_latitude, home_longitude)

  current_altitude = received_msg.alt / 1000

  print(f"ホームロケーションに帰還中（現在位置: 緯度={current_lat}, 経度={current_lon}, 距離={distance_to_home}m, 高度={current_altitude}, 元高度={home_altitude}）")

  # 2メートル以内かつ高度が95%以内に到達あれば帰還とみなす
  if distance_to_home < 2 and current_altitude <= home_altitude * 1.05:
    print("ドローンがホームロケーションに戻りました")
    break

  time.sleep(0.1)


# 切断
master.close()
