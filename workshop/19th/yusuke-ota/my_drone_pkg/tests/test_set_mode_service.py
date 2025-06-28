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
from services.set_mode_service import create_set_mode_service

def main():
    rospy.init_node("test_with_real_mavlink")

    rospy.loginfo("MAVLinkに接続中...")
    master = mavutil.mavlink_connection("127.0.0.1:14551", source_system=255)
    master.wait_heartbeat()
    rospy.loginfo("接続成功: sysid=%d, compid=%d", master.target_system, master.target_component)

    create_set_mode_service(master)
    rospy.wait_for_service("set_mode")

    client = rospy.ServiceProxy("set_mode", SetMode)
    req = SetModeRequest(mode="STABILIZE")
    res = client.call(req)
    print("Result:", res.success, res.message)
    
    rospy.sleep(1)  # 少し待つ
    req = SetModeRequest(mode="LOITER")
    res = client.call(req)
    print("Result:", res.success, res.message)



if __name__ == '__main__':
    main()


