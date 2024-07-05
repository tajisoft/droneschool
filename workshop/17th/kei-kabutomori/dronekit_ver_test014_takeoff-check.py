# 離陸するコードを着陸まで行うトライアルコードに変更
import time
from dronekit import connect, VehicleMode

# UDP接続で確認する
vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)

# 初期化処理
while not vehicle.is_armable:
    print("初期化中です")
    time.sleep(1)

# アーム処理
print("アームします")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("アーム待機中")
    time.sleep(1)

targetAltude = 10 # テスト用に初期値：100m -> 10mに変更

# 離陸処理
print("離陸開始")
vehicle.simple_takeoff(targetAltude)

# 離陸後の高度チェックループ
while True:
    print("高度:",vehicle.location.global_relative_frame.alt)

    if vehicle.location.global_relative_frame.alt >= targetAltude * 0.95:
        print("目標高度に到達しました")
        break

    time.sleep(1)

    # ここまででオリジナルのコマンドは終了

# 目標高度到達後、着陸（Return To Lunch）する処理を追加(Trial) 2024.06.29
print("プログラム終了、1秒後に自動着陸を開始します")
vehicle.mode = VehicleMode("RTL")
time.sleep(1)
