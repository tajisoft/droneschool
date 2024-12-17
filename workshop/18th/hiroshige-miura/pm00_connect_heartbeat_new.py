from pymavlink import mavutil
import time

# connect to UAV drone
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:172.23.16.1:5762", source_system=1, source_component=90)
master.wait_heartbeat()

# display target system ID and component ID
print(f"target_system: {master.target_system}. target_component: {master.target_component}")

# send a HEARTBEAT message every one second
while True:
    time.sleep(1)

    # send a message directly
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0, 0, 0)

