import time
from dronekit import connect, TimeoutError

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

try:
    print("GUIDEDモードにし、ARMします")
    vehicle.wait_for_mode("GUIDED")
    vehicle.wait_for_armable()
    vehicle.arm()
    time.sleep(1)

    print("離陸します")
    vehicle.wait_simple_takeoff(5, timeout=60)

    print("RTLモードにします")
    vehicle.wait_for_mode("RTL")

    print("着陸しDISARMされるのを待ってます")
    while vehicle.armed:
        time.sleep(1)

    print("終了します")

except TimeoutError as takeoffError:
    print("Takeoff is timeout")

