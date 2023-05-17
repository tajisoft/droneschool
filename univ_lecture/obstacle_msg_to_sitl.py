# encoding: utf-8

## Simple obstacle message sending test

import sys
import time
import signal
import threading
import math as m
import numpy as np
import random
from pymavlink import mavutil
from apscheduler.schedulers.background import BackgroundScheduler 

sched = BackgroundScheduler()

DEPTH_RANGE_M = [0.1, 5]

exit_code = 1
main_loop_should_exit = False
mavlink_thread_should_exit = False
vehicle_pitch_rad = None
debug_enable = 1
current_time_us = 0
last_obstacle_distance_sent_ms = None
distances_array_length = 72
min_depth_cm = int(DEPTH_RANGE_M[0] * 100)
max_depth_cm = int(DEPTH_RANGE_M[1] * 100)
increment_f = 87 / distances_array_length
angle_offset = 0 - 87 / 2
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_depth_cm + 1)

## Connect to vehicle
conn = mavutil.mavlink_connection(
    'tcp:192.168.1.5:5763',
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud=115200,
    force_connected=True,
    dialect='ardupilotmega'
)

def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'Test: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    print("INFO: %s" % text_to_be_sent)

def att_msg_callback(value):
    global vehicle_pitch_rad
    vehicle_pitch_rad = value.pitch
    if debug_enable == 1:
        print("INFO: Received ATTITUDE msg, current pitch is %.2f degrees" % (m.degrees(vehicle_pitch_rad),))

## Handling attitude message
mavlink_callbacks = {
    'ATTITUDE': att_msg_callback,
}

## Heartbeat sender
def mavlink_loop(conn, callbacks):
    '''a main routine for a thread; reads data from a mavlink connection,
    calling callbacks based on message type received.
    '''
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
        # send a heartbeat msg
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                0,
                                0,
                                0)
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        callbacks[m.get_type()](m)

## Heartbeat sender thread
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

def send_obstacle_distance_message():
    global conn, distances, min_depth_cm, max_depth_cm, increment_f, angle_offset
    current_time_us = int(round(time.time() * 1000000))
    last_obstacle_distance_sent_ms = current_time_us
    if angle_offset is not None and increment_f is not None:
        print(distances)
        conn.mav.obstacle_distance_send(
            current_time_us,    # us Timestamp (UNIX time or time since system boot)
            0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
            distances,          # distances,    uint16_t[72],   cm
            0,                  # increment,    uint8_t,        deg
            min_depth_cm,	    # min_distance, uint16_t,       cm
            max_depth_cm,       # max_distance, uint16_t,       cm
            increment_f,	    # increment_f,  float,          deg
            angle_offset,       # angle_offset, float,          deg
            12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
        )

## Rate scheduler for send obstacle message
sched.add_job(send_obstacle_distance_message, 'interval', seconds = 1/15)
sched.start()
send_msg_to_gcs('Sending distance sensor messages to FCU')

def sigint_handler(sig, frame):
    global main_loop_should_exit
    main_loop_should_exit = True
signal.signal(signal.SIGINT, sigint_handler)

def sigterm_handler(sig, frame):
    global main_loop_should_exit, exit_code
    main_loop_should_exit = True
    exit_code = 0

## Main thread
try:
    while not main_loop_should_exit:
        print('Generate random distances')
        for n in range(distances_array_length):
            distances[n] = int(random.random() * 5000)
        time.sleep(2)
finally:
    mavlink_thread_should_exit = True
    mavlink_thread.join()
    conn.close()
    sched.remove_all_jobs()
    sched.shutdown()
    print('Finished')

