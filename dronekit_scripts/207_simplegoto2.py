# encoding: utf-8
from dronekit import connect, LocationGlobalRelative, TimeoutError
import time

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

try:
    vehicle.wait_for_armable()
    vehicle.wait_for_mode("GUIDED")
    vehicle.arm()
    time.sleep(1)
    aLocation = LocationGlobalRelative(-35.3574950, 149.1701826, 20)
    vehicle.simple_goto(aLocation, groundspeed=1)

    vehicle.close()
except TimeoutError as takeoffError:
    print("Takeoff is timeout!!!")