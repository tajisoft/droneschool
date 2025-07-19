import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# GUIDEDモードに変更
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name == 'GUIDED':
    print("モード切替待ち...")
    time.sleep(1)

# アーム
vehicle.armed = True
while not vehicle.armed:
    print("アーミング待ち...")
    time.sleep(1)

# 離陸コマンド（高度10m）
target_altitude = 10
print(f"離陸します（目標高度: {target_altitude}m）")
vehicle.simple_takeoff(target_altitude)

# 目標高度に到達するまで待機
while True:
    print(f"現在高度: {vehicle.location.global_relative_frame.alt:.2f}m")
    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("目標高度に到達しました")
        break
    time.sleep(1)

# 安全のため自動でDISARM
print("自動でDISARMします")
vehicle.armed = False
vehicle.close()