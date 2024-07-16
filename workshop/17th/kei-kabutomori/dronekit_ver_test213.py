# Homework day3
from dronekit import Command, connect
from pymavlink import mavutil

vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)

# get command object
cmds = vehicle.commands

# downloard
cmds.download()
cmds.wait_ready()

# keep mission on a spearate list
missionList = []
for cmd in cmds:
    missionList.append(cmd)

# mission edit, 1st mission takeoff
missionList[0].command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

# clear mission
cmds.clear()

# upload mission
for cmd in missionList:
    cmds.add(cmd)
    cmds.upload()