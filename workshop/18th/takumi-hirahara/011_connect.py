import time
from dronekit import connect

# tcp以下は自身の環境に合わせる　TimeOutは60s
vehicle = connect("tcp:192.168.1.57:5762", wait_ready=True, timeout=60)

while True:
    print ("====================================")
    print (f"home_location: {vehicle.home_location}")
    print (f"heading: {vehicle.heading}")
    print (f"gimbal: {vehicle.gimbal}")
    print (f"airspeed: {vehicle.airspeed}")
    print (f"groundspeed: {vehicle.groundspeed}")
    print (f"mode: {vehicle.mode}")
    print (f"armed: {vehicle.armed}")
    time.sleep(1)