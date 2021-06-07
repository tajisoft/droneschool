import time
from dronekit import connect, TimeoutError

vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

try:
    vehicle.wait_for_mode("GUIDED")
    vehicle.wait_for_armable()
    vehicle.arm()
    time.sleep(1)
    vehicle.wait_simple_takeoff(100, timeout=60)
    # vehicle.wait_simple_takeoff(20,0.5,15)

except TimeoutError as takeoffError:
    print("Takeoff is timeout!!!")
    # フェールセーフコード

