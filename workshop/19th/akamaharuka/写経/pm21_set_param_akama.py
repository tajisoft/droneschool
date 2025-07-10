import time
from pymavlink import mavutil

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()

print(
    "Heartbeat from system (system %u component %u)"
    % (master.target_system, master.target_component)
)

# Define the parameter ID for RTL_ALT
param_id = "RTL_ALT"
param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Assuming RTL_ALT is of type REAL32

# Request the current value of the parameter
master.mav.param_request_read_send(
    master.target_system, master.target_component, param_id.encode("ascii"), -1
)

# Wait for the current value of the parameter
while True:
    message = master.recv_match(type="PARAM_VALUE", blocking=True).to_dict()
    if message["param_id"] == param_id:
        current_value = message["param_value"]
        print(f"Current value of {param_id}: {current_value}")
        break

# Define the new value for the parameter
new_value = current_value + 200.0  # Set the desired RTL_ALT value in centimeters

# Send a request to set the parameter
master.mav.param_set_send(
    master.target_system, master.target_component,
    param_id.encode("ascii"), new_value, param_type,
)

# Wait for the ACK of the set parameter request
while True:
    message = master.recv_match(type="PARAM_VALUE", blocking=True).to_dict()
    if message["param_id"] == param_id:
        print(f"Parameter {param_id} set to {message['param_value']}")
        break

# Request the updated value of the parameter
master.mav.param_request_read_send(
    master.target_system, master.target_component, param_id.encode("ascii"), -1
)

# Wait for the updated value of the parameter
while True:
    message = master.recv_match(type="PARAM_VALUE", blocking=True).to_dict()
    if message["param_id"] == param_id:
        updated_value = message["param_value"]
        print(f"Updated value of {param_id}: {updated_value}")
        break

# Close the connection
master.close()
