import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# ARM可能になるまで待機
while not vehicle.is_armable:
    print("初期化中です")
    time.sleep(1)

print("GUIDEDモードに切り替えます")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("アームを待ってます")
    time.sleep(1)

target_altitude = 30

print("離陸！")
vehicle.simple_takeoff(target_altitude)

# 目標高度到達まで監視
while True:
    print("高度:", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("目標高度に到達しました")
        break
    time.sleep(1)

# 5秒ホバリング
print("5秒ホバリングします")
time.sleep(5)

# LANDモードで着陸
print("LANDモードに切り替え、着陸します")
vehicle.mode = VehicleMode("LAND")

# 着陸完了まで監視
while vehicle.armed:
    print("着陸中... 高度:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("着陸完了・DISARMしました")
vehicle.close()