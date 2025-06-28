#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division, absolute_import

from my_drone_pkg.srv import SetArm, SetArmResponse
from pymavlink import mavutil
import rospy

def create_arm_disarm_service(master):
    """
    Create a ROS service to arm or disarm the drone.

    Args:
        master: An instance of the MAVLink master that manages the connection
                to the drone.

    Returns:
        rospy.Service: A ROS service that allows clients to arm or disarm the drone.
    """
    def arm_disarm_callback(req):
        target_state = req.arm  # bool: True to arm, False to disarm

        # 現在のモードを取得してGUIDED以外なら失敗
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if not hb:
            return SetArmResponse(False, "Failed to receive heartbeat")

        current_mode = mavutil.mode_string_v10(hb)
        if current_mode != "GUIDED":
            return SetArmResponse(False, "Current mode is %s, but must be GUIDED to arm/disarm." % current_mode)

        # アーム／ディスアーム実行
        if target_state:
            master.arducopter_arm()
        else:
            master.arducopter_disarm()

        timeout = 5.0
        start_time = rospy.get_time()
        armed_state = None

        while rospy.get_time() - start_time < timeout:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
            if hb:
                armed_state = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if armed_state == target_state:
                    msg = "Drone successfully %s." % ("armed" if target_state else "disarmed")
                    return SetArmResponse(True, msg)

        msg = "Failed to %s the drone. Current state: %s" % (
            "arm" if target_state else "disarm",
            "armed" if armed_state else "disarmed" if armed_state is not None else "unknown"
        )
        return SetArmResponse(False, msg)

    return rospy.Service("set_arm", SetArm, arm_disarm_callback)

