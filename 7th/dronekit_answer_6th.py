from dronekit import connect, VehicleMode
import time

# 期待接続
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# 離陸高度
target_alt = 20
# Guidedモード
guided = VehicleMode('GUIDED')

# モード変更してアーム
vehicle.mode = guided
vehicle.armed = True

# 離陸可能になるまで待機
while True:
    if not vehicle.armed or not vehicle.mode.name == 'GUIDED':
        print('Vehicle not ready...')
        time.sleep(1)

# 離陸
vehicle.simple_takeoff(target_alt)

# ここまでできて入れば可