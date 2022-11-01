import time
from dronekit import Vehicle, connect

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

#RTL_ALTが変更された際に通知を受け取る
#デコレーターの設定
@vehicle.parameters.on_attribute('RTL_ALT')
def decorated_RTL_ALT_callback(self, attr_name, value):
	print("デコレーター: %s が %s に変更されました！" % (attr_name, value))

while True:
    time.sleep(1)