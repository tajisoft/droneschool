# encoding: utf-8
import sys
import time

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

vehicles = None

def heartbeat_handler(self, name, val):
    print(self)
    print(name)
    print(val)

def prepare_vehicles():
    plane = connect('tcp:127.0.0.1:5763', wait_ready=True, timeout=60)
    copter = connect('tcp:127.0.0.1:5766', wait_ready=True, timeout=60)
    boat = connect('tcp:127.0.0.1:5769', wait_ready=True, timeout=60)
    rover1 = connect('tcp:127.0.0.1:5772', wait_ready=True, timeout=60)
    rover2 = connect('tcp:127.0.0.1:5775', wait_ready=True, timeout=60)
    plane.add_message_listener('HEARTBEAT', heartbeat_handler)
    copter.add_message_listener('HEARTBEAT', heartbeat_handler)
    boat.add_message_listener('HEARTBEAT', heartbeat_handler)
    rover1.add_message_listener('HEARTBEAT', heartbeat_handler)
    rover2.add_message_listener('HEARTBEAT', heartbeat_handler)
    return [plane, copter, boat, rover1, rover2]

def launch_vehicle(vehicle):
    pass