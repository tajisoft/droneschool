# encoding: utf-8
# パッケージインポート
from dronekit import connect, VehicleMode
import time

# 接続（例：ローカルPCで動作しているシミュレータ
vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)

# GUIDEDモードにしてアーム
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

timeout = 10
start = time.time()

# 指示した状態になるまでループでチェック
# 実際の実装ではフェールセーフになるようにしてください。
while not vehicle.mode.name == "GUIDED" or not vehicle.armed:
    if ((time.time() - start) >= timeout):
        break # タイムアウトとなり、処理失敗とみなし、失敗処理へ移行しましょう
    time.sleep(1)

print("テイクオフ")
targetAltitude = 20
vehicle.simple_takeoff(targetAltitude)

# 目標高度になるまでまつ
while True:
    if vehicle.location.global_relative_frame.alt >= targetAltitude*0.95:
        print("離陸高度に達しました")
        break
    time.sleep(1)

