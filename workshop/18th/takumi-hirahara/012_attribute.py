import time
from dronekit import connect, VehicleMode

# tcp以下は自身の環境に合わせる　TimeOutは60s
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# GUIDEDモード変更
vehicle.mode = VehicleMode("GUIDED")

# アーム
vehicle.armed = True

# グラウンドスピードを3.2m/sに設定
vehicle.groundspeed = 3.2

# モードがGUIDEDになり、ARMされるまで待つ
while not vehicle.mode.name == 'GUIDED' and not vehicle.armed:
    time.sleep(1)