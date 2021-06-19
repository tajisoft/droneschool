import time
from dronekit import  connect

# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)

print("次のミッションを表示するにはEnterキーを押してください。")

while True:
    time.sleep(1)
    if input() == "":
        print("次のミッションは: %s" % vehicle.commands.next)