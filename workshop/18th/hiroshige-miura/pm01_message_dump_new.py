from pymavlink import mavutil
import time

# connect to UAV drone
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:172.23.16.1:5762", source_system=1, source_component=90)
master.wait_heartbeat()

# receive messages
receive_msg = master.recv_match(type='HEARTBEAT', blocking=True)
print(receive_msg)

# send GLOBAL_POSITION_INT(33) message at 10Hz
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0)

while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(1.0)



