import sys
import time
from dronekit import connect, VehicleMode

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# arm不可能なモードもしくはセーフティロックがかかっている場合はこの処理でスタックする可能性があります
while not vehicle.is_armable:
    print("初期化中です")
    time.sleep(1)

# GUIDEDを外側から制御することにしてGUIDEDになるのを待つ
# 但し制限付き
cnt = 10
while vehicle.mode != VehicleMode("GUIDED") :
    if cnt > 0 :
      cnt = cnt - 1
      time.sleep(1)
    else :
      print("GUIDEDになかなか変更されないので終わり！！")
      sys.exit(1)

print("GUIDEDに変更されたのでアームします")
#vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("アームを待ってます")
    time.sleep(1)

targetAltude = 100

print("離陸！")
vehicle.simple_takeoff(targetAltude)

while True:
    print("高度:",vehicle.location.global_relative_frame.alt)

    if vehicle.location.global_relative_frame.alt >= (targetAltude - 1) :
        print("目標高度に到達しました")
        break

    time.sleep(1)

# しばらくホバリング
print("ホバリング中")
time.sleep(10)

# 着陸
print("着陸します")
vehicle.mode = VehicleMode("LAND")
time.sleep(5) # 着陸発動まで少し待つ
cnt = 5
lastAlt = vehicle.location.global_relative_frame.alt
while True:
    time.sleep(1)
    nowAlt = vehicle.location.global_relative_frame.alt
    print("高度:",nowAlt)

    if lastAlt == nowAlt :
        if cnt > 0 :
            cnt = cnt - 1
        else :
            print("着陸しました")
            break
    else :
        cnt = 5
    lastAlt = nowAlt

