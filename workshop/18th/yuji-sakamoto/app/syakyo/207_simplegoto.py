# このコードは離陸した後で実行してください。
# SITLのロケーション設定を奥多摩の練習場に設定（35.81168785077643, 139.15884549638795）
# => ardupilot/Tools/autotest/locations.txtに設定を追加して対応
# SITL側で以下を実行後に本スクリプトを実行
# > mode alt_hold
# > rc 3 1000
# > arm throttle
# > takeoff 5
# > rc 3 1600
# 実行させて飛行後、RTLで戻す

import time
from dronekit import LocationGlobalRelative, VehicleMode, connect

#vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# モードはGUIDED
vehicle.mode = VehicleMode("GUIDED")

# 目標の緯度・経度、高度を設定する
# https://maps.gsi.go.jp/#8/-35.3574950/149.1701826/&base=std&ls=std&disp=1&vs=c1j0h0k0l0u0t0z0r0s0m0f1
#aLocation = LocationGlobalRelative(-35.3574950, 149.1701826, 20)
# 奥多摩の旧古里中学校グラウンド座標を設定
aLocation = LocationGlobalRelative(35.812333, 139.158605, 20)

# simple_gotoを実行する
vehicle.simple_goto(aLocation, groundspeed=1000, airspeed=1000)

# wait by time
time.sleep(10)
# 接続を閉じる
vehicle.close()
