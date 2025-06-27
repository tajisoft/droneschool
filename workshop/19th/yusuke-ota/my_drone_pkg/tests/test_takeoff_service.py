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
from my_drone_pkg.srv import SetTakeoff, SetTakeoffRequest, SetTakeoffResponse
from services.set_mode_service import create_set_mode_service
from services.arm_disarm_service import create_arm_disarm_service
from services.takeoff_service import create_takeoff_service

def main():
    rospy.init_node("test_takeoff_with_real_mavlink")

    rospy.loginfo("MAVLinkに接続中...")
    master = mavutil.mavlink_connection("127.0.0.1:14551", source_system=255)
    master.wait_heartbeat()
    rospy.loginfo("接続成功: sysid=%d, compid=%d", master.target_system, master.target_component)

    # サービス登録
    create_set_mode_service(master)
    create_arm_disarm_service(master)
    create_takeoff_service(master)

    rospy.wait_for_service("set_mode")
    rospy.wait_for_service("set_arm")
    rospy.wait_for_service("takeoff")

    # GUIDEDモードに変更
    set_mode_client = rospy.ServiceProxy("set_mode", SetMode)
    set_mode_req = SetModeRequest(mode="GUIDED")
    set_mode_res = set_mode_client.call(set_mode_req)
    print("GUIDED Mode Result:", set_mode_res.success, set_mode_res.message)

    # アーム要求
    arm_client = rospy.ServiceProxy("set_arm", SetArm)
    arm_req = SetArmRequest(arm=True)
    arm_res = arm_client.call(arm_req)
    print("Arm Result:", arm_res.success, arm_res.message)

    rospy.sleep(1.0)  # 少し待つ

    # テイクオフ要求（高度3.0m）
    takeoff_client = rospy.ServiceProxy("takeoff", SetTakeoff)
    takeoff_req = SetTakeoffRequest(altitude=3.0, timeout=10.0, epsilon=0.2)
    takeoff_res = takeoff_client.call(takeoff_req)
    print("Takeoff Result:", takeoff_res.success, takeoff_res.message)

    rospy.sleep(2.0)  # 飛行中の待機（高度到達を想定）

    # LANDモードに変更
    set_mode_client = rospy.ServiceProxy("set_mode", SetMode)
    set_mode_req = SetModeRequest(mode="LAND")
    set_mode_res = set_mode_client.call(set_mode_req)
    print("Land Mode Result:", set_mode_res.success, set_mode_res.message)

    rospy.sleep(8.0)  # 着陸するまで待機（必要に応じて調整）


if __name__ == '__main__':
    main()
