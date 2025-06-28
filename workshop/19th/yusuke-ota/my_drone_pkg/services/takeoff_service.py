#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division, absolute_import

from my_drone_pkg.srv import SetTakeoff, SetTakeoffResponse
from pymavlink import mavutil
import rospy

def create_takeoff_service(master):
    """
    Create a ROS service to initiate drone takeoff to a specified altitude.
    Waits until the drone reaches the target altitude or timeout.

    Args:
        master: An instance of the MAVLink master that manages the connection
                to the drone.

    Returns:
        rospy.Service: A ROS service that allows clients to command takeoff.
    """
    def takeoff_callback(req):
        target_alt = req.altitude
        timeout = req.timeout
        epsilon = req.epsilon

        # 現在のモードを取得してGUIDED以外なら失敗
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if not hb:
            return SetTakeoffResponse(False, "Failed to receive heartbeat")

        current_mode = mavutil.mode_string_v10(hb)
        if current_mode != "GUIDED":
            return SetTakeoffResponse(False, "Current mode is %s, but must be GUIDED to takeoff." % current_mode)

        start_time = rospy.get_time()
        armed_state = None

        while rospy.get_time() - start_time < 5.0:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if hb:
                armed_state = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                break

        if armed_state is None:
            return SetTakeoffResponse(False, "Failed to receive heartbeat")

        if not armed_state:
            master.arducopter_arm()
            rospy.sleep(1.0)
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if not hb or not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return SetTakeoffResponse(False, "Failed to arm the drone")

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0,
            target_alt
        )

        start_time = rospy.get_time()
        current_alt = -1.0

        while rospy.get_time() - start_time < timeout:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
            if msg:
                current_alt = msg.relative_alt / 1000.0
                if current_alt >= (target_alt - epsilon):
                    return SetTakeoffResponse(True, "Takeoff completed. Reached altitude: %.2f m" % current_alt)

        return SetTakeoffResponse(False, "Takeoff timeout. Current altitude: %.2f m" % current_alt)

    return rospy.Service("takeoff", SetTakeoff, takeoff_callback)

