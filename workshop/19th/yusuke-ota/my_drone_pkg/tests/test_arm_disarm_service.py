#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division, absolute_import
import sys
import os
# モジュールパスに追加
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import rospy
from pymavlink import mavutil
from my_drone_pkg.srv import SetMode, SetModeRequest, SetModeResponse
from my_drone_pkg.srv import SetArm, SetArmRequest, SetArmResponse
from services.set_mode_service import create_set_mode_service
from services.arm_disarm_service import create_arm_disarm_service

def main():
    rospy.init_node("test_arm_disarm_with_real_mavlink")

    rospy.loginfo("MAVLinkに接続中...")
    master = mavutil.mavlink_connection("127.0.0.1:14551", source_system=255)
    master.wait_heartbeat()
    rospy.loginfo("接続成功: sysid=%d, compid=%d", master.target_system, master.target_component)

    create_arm_disarm_service(master)
    create_set_mode_service(master)
    rospy.wait_for_service("set_arm")
    rospy.wait_for_service("set_mode")

    client = rospy.ServiceProxy("set_arm", SetArm)

    # GUIDEDモードに変更
    set_mode_client = rospy.ServiceProxy("set_mode", SetMode)
    set_mode_req = SetModeRequest(mode="GUIDED")
    set_mode_res = set_mode_client.call(set_mode_req)
    print("GUIDED Mode Result:", set_mode_res.success, set_mode_res.message)

    # アーム要求
    req = SetArmRequest(arm=True)
    res = client.call(req)
    print("Arm Result:", res.success, res.message)

    rospy.sleep(2)  # アーム状態で2秒待機

    # ディスアーム要求
    req = SetArmRequest(arm=False)
    res = client.call(req)
    print("Disarm Result:", res.success, res.message)

if __name__ == '__main__':
    main()
