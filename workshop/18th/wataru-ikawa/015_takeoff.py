import time
from dronekit import connect, VehicleMode

vehicle = connect("tcp:192.168.1.57:5762", wait_ready=True, timeout=60)

# arm不可能なモードもしくはセーフティロックがかかっている場合はこの処理でスタックする可能性があります
while not vehicle.is_armable:
    print("初期化中です")
    time.sleep(1)

print("アームします")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("アームを待ってます")
    time.sleep(1)

targetAltude = 10

print("離陸！")
vehicle.simple_takeoff(targetAltude)

while True:
    print(f"高度: {vehicle.location.global_relative_frame.alt}")

    if vehicle.location.global_relative_frame.alt >= targetAltude * 0.95:
        print("目標高度に到達しました")
        break

    time.sleep(1)
