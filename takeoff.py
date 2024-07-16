from dronekit import connect, VehicleMode
import time

# 機体接続
vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)

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
vehicle.wait_simple_takeoff(target_alt, epsilon=0.5, timeout=20)

# Wait
time.sleep(10)

# RTL
vehicle.wait_for_mode(VehicleMode("RTL"))

time.sleep(10)

# TODO 着陸チェック
vehicle.close()
# ここまでできて入れば可

print("Script finished")
