from pymavlink import mavutil

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    device="127.0.0.1:14551", source_system=1, source_component=90
)

# HEARTBEATをまつ
master.wait_heartbeat()

# ヨー制御コマンドメッセージを作成する場合
msg = master.mav.command_long_encode(
    0, 1,    # ターゲットシステム、コンポーネント
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,   # コマンド
    0,
    180,     # 角度指定(deg)
    0,       # スピード指定(deg/s)
    1,       # 方向　-1:CCW, 1:CW
    1,       # offset 1:relative, 0:absolute
    0, 0, 0  #
)

# MAVLinkメッセージ送信
master.mav.send(msg)