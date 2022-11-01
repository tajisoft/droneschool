from dronekit import  connect
import time

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

@vehicle.on_message('RANGEFINDER')
def listener(self, name, message):
    print('distance: %s' % message.distance)
    print('voltage: %s' % message.voltage)

time.sleep(10)

# このコードをテストするにはSITLで下記コマンドを実行してRangefinderを有効にしてください。
# > param set RNGFND1_TYPE 100
# > reboot
# 
# 下記コマンドでRangefinderの値のグラフ化が可能です。
# > module load graph
# > graph RANGEFINDER.distance
