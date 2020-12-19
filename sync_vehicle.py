# encoding: utf-8
import sys
import time

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

myVehicle = None
remoteVehicle = None


def connect_vehicle(dest_str):
    return connect(dest_str, wait_ready = True)

def arm_handler(self, attr, val):
    print(val)
    print('{} {}'.format(attr, val))
    if myVehicle.armed != val:
        myVehicle.wait_for_armable()
        myVehicle.arm()
        myVehicle.wait_for_mode('GUIDED')

def mode_handler(self, attr, val):
    print(val)
    print('{} {}'.format(attr, val))
    if myVehicle.mode.name != val.name and myVehicle.mode.name != 'GUIDED':
        myVehicle.mode = val

def pos_handler(self, attr, val):
    print(val)
    if val.alt > 0.5 and not myVehicle.armed:
        takeoff(myVehicle, 5)

def vel_handler(self, attr, val):
    print(val)
    vel_sync(myVehicle, val, remoteVehicle.attitude.yaw)

def isReady(v):
    return v.armed and v.mode.name == 'GUIDED' and v.location.global_relative_frame.alt > 0.5

def arm(v):
    v.wait_for_mode('GUIDED')
    v.wait_for_armable()

def takeoff(v, target_alt):
    arm(v)
    v.arm()
    v.wait_simple_takeoff(target_alt)

def vel_sync(v, vel, yaw):
    if isReady(v):
        msg = v.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000101111000111,
            0, 0, 0,
            vel[0], vel[1], vel[2],
            0, 0, 0,
            yaw, 0)
        v.send_mavlink(msg)

def start_monitor_and_sync():
    remoteVehicle.add_attribute_listener('armed', arm_handler)
    remoteVehicle.add_attribute_listener('mode', mode_handler)
    remoteVehicle.add_attribute_listener('location.global_relative_frame', pos_handler)
    remoteVehicle.add_attribute_listener('velocity', vel_handler)
    # Check alive
    while remoteVehicle.last_heartbeat < 10:
        print('last_heartbeat {}'.format(remoteVehicle.last_heartbeat))
        print(remoteVehicle.velocity)
        time.sleep(1)


if __name__ == "__main__":
    myVehicle = connect_vehicle("tcp:192.168.11.16:5773")
    remoteVehicle = connect_vehicle("tcp:192.168.11.16:5783")
    start_monitor_and_sync()
    