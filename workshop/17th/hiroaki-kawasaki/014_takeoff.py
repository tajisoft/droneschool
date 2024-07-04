import time
from dronekit import connect, VehicleMode

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# シミュレーター起動直後は１分程度ウエイトする場合がある
start = time.time()
while not vehicle.is_armable:
    elapsed = time.time() - start
    print(f"{elapsed:.1f}: 初期化中です")
    time.sleep(1)

elapsed = time.time() - start
print(f"{elapsed:.1f}: アームします")

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    elapsed = time.time() - start
    print(f"{elapsed:.1f}: アームを待ってます")
    time.sleep(1)

targetAltude = 5

elapsed = time.time() - start
print(f"{elapsed:.1f}: 離陸")
vehicle.simple_takeoff(targetAltude)

while True:
    elapsed = time.time() - start
    print(f"{elapsed:.1f}: 高度 {vehicle.location.global_relative_frame.alt}")

    if vehicle.location.global_relative_frame.alt >= targetAltude * 0.95:
        print(f"{elapsed:.1f}: 目標高度に到達しました")
        break
    time.sleep(1)
    
elapsed = time.time() - start
print(f"{elapsed:.1f}: RTLモードに遷移します")
vehicle.mode = VehicleMode("RTL")

while vehicle.armed:
    elapsed = time.time() - start
    print(f"{elapsed:.1f}: ディスアームを待ってます")
    time.sleep(1)

elapsed = time.time() - start
print(f"{elapsed:.1f}: 終了します")
