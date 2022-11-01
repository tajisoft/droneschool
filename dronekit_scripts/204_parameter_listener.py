
import time
from dronekit import connect

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)


# コールバック関数
def RTL_ALT_parameter_callback(self, attr_name, value):
	print("リスナー: %s が %s に変更されました！"  % (attr_name, value))


# コールバック関数「RTL_ALT_parameter_callback」を「RTL_ALT」変更通知用に登録する
vehicle.parameters.add_attribute_listener(
    'RTL_ALT', RTL_ALT_parameter_callback)

while True:
    time.sleep(1)