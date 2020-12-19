from dronekit import connect, VehicleMode
import time

# 期待接続
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# 離陸高度
target_alt = 10
# Guidedモード
guided = VehicleMode('GUIDED')

# モード変更してアーム
vehicle.wait_for_armable(30)
print("Vehicle is armable")
vehicle.wait_for_mode(guided, timeout=5)
print("Mode changed")
vehicle.arm(wait=True, timeout=5)
print("Armed state {}".format(vehicle.armed))

# 離陸
print("takeoff")
vehicle.wait_simple_takeoff(target_alt, timeout=10)

# Loiter
vehicle.wait_for_mode(VehicleMode("LOITER"))

vehicle.close()
# ここまでできて入れば可