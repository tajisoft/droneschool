from dronekit import connect
from pymavlink import mavutil

# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)

# ヨー制御コマンドメッセージを生成する場合
msg = vehicle.message_factory.command_long_encode(
    0, 1,    # ターゲットシステム、コンポーネント
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # コマンド
    0,
    180,    # 角度指定（degrees）
    0,      # スピード指定（deg/s）
    1,      # 方向 -1:反時計周り, 1:時計回り
    1,      # オフセット 1:相対, 0:絶対
    0, 0, 0)

# MAVLinkメッセージ送信
vehicle.send_mavlink(msg)
