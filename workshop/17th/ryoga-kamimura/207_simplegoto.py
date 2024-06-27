#このコードで離陸から実行できるように変更しました
import time

from dronekit import LocationGlobalRelative, VehicleMode, connect

#SITLとの接続アドレス
CONNECTION_STRING = 'tcp:127.0.0.1:5762'


vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
#arm可能になるのを待つコードを追加
while not vehicle.is_armable:
    print('Waiting for vehicle to become armable')
    time.sleep(1)

#モードはGUIDED
vehicle.mode = VehicleMode('GUIDED')
#モードが変わるまで待機するコードを追加
while vehicle.mode != 'GUIDED':
    print('Change mode to GUIDED')
    time.sleep(1)

#アームするコードを追加
vehicle.armed = True
while not vehicle.armed:
    print('Waiting for arming')
    time.sleep(1)

#離陸するするコードを追加
print('Taking off')
vehicle.simple_takeoff(10)

#少し待つ
time.sleep(10)

#目標の緯度・経度、高度を設定する
aLocation = LocationGlobalRelative(-35.3574950, 149.1701826, 20)

#simple_gotoを実行する
vehicle.simple_goto(aLocation, groundspeed=1000, airspeed=1000)