import time
from dronekit import connect
from pymavlink import mavutil

CONNECTION_STRING = 'tcp:127.0.0.1:5762'

vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)

msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,      #ブートからの時間(今回は未使用)
    0,0,    #ターゲットシステム、コンポーネント
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, #フレーム
    0b0000111111000111, #タイプマスク 0:有効 1:無効
    0,0,0,  #x,y,z位置(今回は未使用)
    2,-2,1, #x,y,z速度(m/s)
    0,0,0,  #x,y,z加速度
    0,0     #ヨー, ヨーレート
)

for x in range(0, 100):
    vehicle.send_mavlink(msg)
    time.sleep(0.1)