# Simple sample

import time
import math
from dronekit import Vehicle, connect, VehicleMode, Command
from pymavlink import mavutil
import pymap3d as pm
import numpy as np
###----------------------------- 1. connect multiple drones   -------------------------------------------------
###----------------------------- 2. find the nearest location -------------------------------------------------
# reference : https://qiita.com/ina111/items/6e3c4d85036fd993d23c
"""
ad = ['tcp:143.189.114.165:5862',
      'tcp:143.189.114.165:5872',
      'tcp:143.189.114.165:5882',
      'tcp:143.189.114.165:5892',
      'tcp:143.189.114.165:5902']
"""
ad = ['tcp:143.189.114.165:5892']

i = 0
distances = np.zeros(len(ad))
vehicle_list = []
lat_goal = 35.806627
lon_goal = 139.085252
alt_goal = 0.0

alt_input = 300

while len(ad) > i:
  vehicle = connect(ad[i], wait_ready=True, timeout=60)
  vehicle_list.append(vehicle)

  alt_goal = vehicle_list[i].location.global_frame.alt

  lat_home = vehicle_list[i].location.global_frame.lat
  lon_home = vehicle_list[i].location.global_frame.lon
  alt_home = vehicle_list[i].location.global_frame.alt
  
  # input goal location

  
  az,el,range = pm.geodetic2aer(lat_goal, lon_goal, alt_goal, lat_home, lon_home, alt_home)
  
  print(ad[i])
  print("home to goal : 方位角 = %.1f [deg], 仰角 = %.1f [deg], 直線距離 = %.1f [m]" % (az, el, range))
  distances[i] = range
  #vehicle.close
  #del vehicle
  time.sleep(2)
  i = i + 1

index_min = np.argmin(distances)
print("The nearest drone is :", ad[index_min])
print(vehicle_list)

###----------------------------- 3. write mission-------------------------------------------------
# reference : https://github.com/dronekit/dronekit-python/blob/master/examples/mission_basic/mission_basic.py

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle_list[index_min].is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle_list[index_min].mode = VehicleMode("GUIDED")
    vehicle_list[index_min].armed = True

    while not vehicle_list[index_min].armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle_list[index_min].simple_takeoff(aTargetAltitude) # Take off to target altitude
    time.sleep(15)
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    """
    while True:
        print(" Altitude: ", vehicle_list[index_min].location.global_relative_frame.alt)      
        if vehicle_list[index_min].location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
    """
def adds_mission():
    cmds = vehicle_list[index_min].commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt_input))
    lat_goal = 35.806627
    lon_goal = 139.085252
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 35.8051104, 139.0826440, alt_input))
    #add dummy waypoint "2" at point 1 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat_goal, lon_goal, alt_input))    

    print(" Upload new commands to vehicle")
    cmds.upload()

# set parameters
vehicle_list[index_min].parameters['RTL_ALT'] = alt_input# + 500 
vehicle_list[index_min].parameters['RTL_ALT_TYPE'] = 1
vehicle_list[index_min].parameters['WPNAV_SPEED'] = 2000 
vehicle_list[index_min].parameters['WPNAV_SPEED_UP'] = 1000
vehicle_list[index_min].parameters['WPNAV_SPEED_DOWN'] = 400
vehicle_list[index_min].parameters['RTL_SPEED'] = 2000  
vehicle_list[index_min].parameters['FENCE_ENABLE'] = 1
vehicle_list[index_min].parameters['FENCE_ACTION'] = 0
print('Create a new mission (for current location)')
adds_mission()

arm_and_takeoff(alt_input)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle_list[index_min].commands.next=0

# Set mode to AUTO to start mission
vehicle_list[index_min].mode = VehicleMode("AUTO")

while True:
    nextwaypoint=vehicle_list[index_min].commands.next
    print('Running! next point is:',nextwaypoint)
    print('Current location',vehicle_list[index_min].location.global_frame)
    """
    if nextwaypoint>=2: #Dummy waypoint - as soon as we reach waypoint 1 this is true and we exit.
        print("Exit 'standard' mission when start heading to final waypoint (2)")
        break
    """
    alt_goal = vehicle_list[index_min].location.global_frame.alt
    lat_home = vehicle_list[index_min].location.global_frame.lat
    lon_home = vehicle_list[index_min].location.global_frame.lon
    alt_home = vehicle_list[index_min].location.global_frame.alt
    # input goal location
    az,el,range = pm.geodetic2aer(lat_goal, lon_goal, alt_goal, lat_home, lon_home, alt_home)
    #print(ad[i])
    print("home to gola : 方位角 = %.1f [deg], 仰角 = %.1f [deg], 直線距離 = %.1f [m]" % (az, el, range))
    #distances[i] = range
    if range <= 50:
        print("Exit")
        break
    time.sleep(1)

###----------------------------- 4. circle mode -------------------------------------------------
print('Circle mode running')
vehicle_list[index_min].parameters['CIRCLE_RADIUS'] = 5000
time.sleep(0.1)
vehicle_list[index_min].parameters['CIRCLE_RATE'] = 3
time.sleep(0.1)
vehicle_list[index_min].parameters['CIRCLE_OPTIONS'] = 0
time.sleep(0.1)

### TODO Object Avoidance-------------------------------------------------
vehicle_list[index_min].parameters['OA_TYPE'] = 2
vehicle_list[index_min].parameters['OA_MARGIN_MAX'] = 10
### \TODO Object Avoidance-------------------------------------------------

time.sleep(0.1)
vehicle.channels.overrides['3'] = 1500
vehicle_list[index_min].mode = VehicleMode("CIRCLE")

start = time.time()
t = time.time() - start
while t <= 120.0:
  vehicle_list[index_min].channels.overrides['3'] = 1500
  t = time.time() - start

###----------------------------- 5. RTL mode -------------------------------------------------
print('Return to launch')
vehicle_list[index_min].mode = VehicleMode("RTL")

time.sleep(5)
while not vehicle_list[index_min].mode.name == 'RTL':
  time.sleep(1)
  vehicle_list[index_min].mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle_list[index_min].close()