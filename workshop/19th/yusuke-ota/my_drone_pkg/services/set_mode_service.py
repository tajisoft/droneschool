#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division, absolute_import

from my_drone_pkg.srv import SetMode, SetModeResponse
from pymavlink import mavutil
import rospy

def create_set_mode_service(master):
    """
    Create a ROS service to set the mode of a drone.

    Args:
        master: An instance of the MAVLink master that manages the connection
                to the drone.

    Returns:
        rospy.Service: A ROS service that allows clients to set the drone's mode.
    """
    def set_mode_callback(req):
        target_mode = req.mode.upper()
        try:
            mode_id = master.mode_mapping()[target_mode]
        except KeyError:
            return SetModeResponse(False, "Unknown mode: %s" % target_mode)

        master.set_mode(mode_id)

        timeout = 5.0
        start_time = rospy.get_time()
        current_mode = "UNKNOWN"

        while rospy.get_time() - start_time < timeout:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
            if hb:
                current_mode = mavutil.mode_string_v10(hb)
                if current_mode == target_mode:
                    return SetModeResponse(True, "Mode changed to: %s" % target_mode)

        return SetModeResponse(False, "Failed to change mode. Current mode: %s" % current_mode)

    return rospy.Service("set_mode", SetMode, set_mode_callback)
