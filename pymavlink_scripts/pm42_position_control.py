from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    device="127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()

# MAVLinkメッセージ作成
msg = master.mav.set_position_target_local_ned_encode(
    0,          # ブートからの時間（今回は未使用）
    0, 0,       # ターゲットシステム、コンポーネント
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # フレーム
    0b0000111111000111,  # タイプマスク 0:有効, 1:無効
    0, 0, 0,    # x, y, z位置（今回は未使用）
    2, -2, -1,   # x, y, z速度m/s
    0, 0, 0,    # x, y, z加速度（今回は未使用）
    0, 0)       # ヨー, ヨーレート

# MAVLinkメッセージ送信
for x in range(0, 20):
    master.mav.send(msg)
    time.sleep(0.1)
