import time
from dronekit import Vehicle, connect

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:10.0.2.135:5762', wait_ready=True, timeout=60)

# コールバック関数
def location_callback(self, attr_name, value):
    print("Location (Global):", value)


# コールバック関数「location_callback」を「global_frame」変更通知用に登録する
vehicle.add_attribute_listener('location.global_frame', location_callback)

# コールバック関数が解除されるまでの10秒間「Location (Global):」が表示される
time.sleep(10)

# 登録したコールバック関数を解除します
vehicle.remove_attribute_listener("location.global_frame", location_callback)

time.sleep(2)
