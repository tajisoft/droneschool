import time
from dronekit import Vehicle, connect

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# パラメータの参照
# print文でパラメータを表示
print("変更前 RTL_ALT: %s" % vehicle.parameters['RTL_ALT'])

# パラメータ更新
# スロットル最小値を100に更新
vehicle.parameters['RTL_ALT'] = 100

print("変更後 RTL_ALT: %s" % vehicle.parameters['RTL_ALT'])

time.sleep(5)

# パラメータの参照
# イテレータにて順print文で「Key:Value」を表示
for key, value in vehicle.parameters.items():
    print("Key:%s Value:%s" % (key, value))
