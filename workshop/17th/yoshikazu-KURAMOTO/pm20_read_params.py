import time
import sys

# Import mavutil
from pymavlink import mavutil

# Create the connection(機体への接続)
master : mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90
 )

 # Wait a heartbeat before sending commands(HEARTBEATを待つ)
 master.wait_heartbeat()

# Request all parameters
master.mav.param_request_list_send(
    master.target_system, master.target_component
)

while True:
    time.sleep(0.01)
    try:
        message = master.recv_match(
            type='PARAM_VALUE'
        )