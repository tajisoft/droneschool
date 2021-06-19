from dronekit import Command, connect
from pymavlink import mavutil

# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)

# コマンドオブジェクトの取得
cmds = vehicle.commands

# ダウンロード実行
cmds.download()
cmds.wait_ready()

# コマンド定義（テイクオフミッションを定義）
cmd = Command(
    0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 0, 10
)

# コマンド追加
cmds.add(cmd)
# ミッションアップロード
cmds.upload()
