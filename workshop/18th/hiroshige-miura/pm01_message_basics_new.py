from pymavlink import mavutil

# connect to UAV drone
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:172.23.16.1:5762", source_system=1, source_component=90)
master.wait_heartbeat()

# receive messages
receive_msg = master.recv_match(type='HEARTBEAT', blocking=True)
print(receive_msg)

# send a message directly
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    0, 0, 0)

# encode and send messages
to_send_msg = master.mav.heartbeat_encode(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    0, 0, 0)
print(to_send_msg)
master.mav.send(to_send_msg)
