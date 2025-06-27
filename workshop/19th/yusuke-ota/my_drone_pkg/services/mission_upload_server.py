#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division, absolute_import

import rospy
from geometry_msgs.msg import Point
from my_drone_pkg.srv import UploadMission, UploadMissionResponse
from pymavlink import mavutil

def create_upload_service(master):
    def upload_cb(req):
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        rospy.sleep(1.0)

        # waypoint一覧をキャッシュ
        waypoints = [
            (int(wp.x * 1e7), int(wp.y * 1e7), wp.z)
            for wp in req.waypoints
        ]
        count = len(waypoints)

        master.mav.mission_count_send(master.target_system, master.target_component, count)

        for _ in range(count):
            msg = master.recv_match(type='MISSION_REQUEST_INT', blocking=True, timeout=5.0)
            if not msg:
                return UploadMissionResponse(False, "MISSION_REQUEST_INT timeout")

            seq = msg.seq
            lat, lon, alt = waypoints[seq]
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0 if seq != 0 else 1,  # current
                1, 0, 0, 0, 0,         # autocontinue, param1-4
                lat, lon, alt
            )

        ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5.0)
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return UploadMissionResponse(True, "Uploaded {} waypoints.".format(count))
        else:
            return UploadMissionResponse(False, "Upload failed. ACK: {}".format(getattr(ack, 'type', 'None')))
    return rospy.Service("upload_mission", UploadMission, upload_cb)


if __name__ == "__main__":
    rospy.init_node("mission_upload_server")
    master = mavutil.mavlink_connection('127.0.0.1:14551',source_system=255)
    master.wait_heartbeat()
    rospy.sleep(2)
    create_upload_service(master)
    rospy.loginfo("Mission upload service is ready.")
    rospy.spin()
