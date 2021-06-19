from dronekit import  connect
import time

# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)

msg = vehicle.message_factory.distance_sensor_encode(
    0,0,1000,
    120,    # cm
    0,      # Laser
    0,0,0
)

print(msg)

while True:
    vehicle.send_mavlink(msg)
    time.sleep(0.2)