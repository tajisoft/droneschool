from dronekit import Vehicle, connect
import time

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# vehicle.location.global_frameの変更通知を受け取る
@vehicle.on_attribute('location.global_frame')
def location_callback(self, attr_name, value):
    print("Location (Global):", value)

# 10秒間「Location (Global):」が表示される
time.sleep(10)

# 解除する方法はないのでCtrl+C で終了