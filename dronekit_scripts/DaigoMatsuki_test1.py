import time
from dronekit import connect

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
vehicle = connect('tcp:10.40.242.238:5762', wait_ready=True, timeout=60)

def print_vehicle_status(vehicle):
    print("====================================")
    print(f"home_location: {vehicle.home_location}")
    print(f"heading: {vehicle.heading}")
    print(f"gimbal: {vehicle.gimbal}")
    print(f"airspeed: {vehicle.airspeed}")
    print(f"groundspeed: {vehicle.groundspeed}")
    print(f"mode: {vehicle.mode}")
    print(f"armed: {vehicle.armed}")
    #表示項目の追加
    print(f"battery: {vehicle.battery}")
    print(f"location.global_frame: {vehicle.location.global_frame}")
    print(f"location.global_relative_frame: {vehicle.location.global_relative_frame}")
#例外処理の追加
try:
    while True:
        print_vehicle_status(vehicle)
        time.sleep(1)
except KeyboardInterrupt:
    print("終了します。")
    vehicle.close()