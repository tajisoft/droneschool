#!/usr/bin/env python
from __future__ import print_function, division, absolute_import

import rospy
from geopy.distance import distance
from geopy import Point
from geometry_msgs.msg import Point as GeoPoint
from my_drone_pkg.srv import UploadMission

def generate_figure_eight(center_lat, center_lon, center_alt, radius_m):
    center = Point(center_lat, center_lon)
    res = max(12, int(2 * 3.1415 * radius_m))
    points = []

    left = distance(meters=radius_m).destination(center, 270)
    for i in range(res):
        theta = 360 * i / res
        pos = distance(meters=radius_m).destination(left, theta)
        points.append(GeoPoint(pos.latitude, pos.longitude, center_alt))

    right = distance(meters=radius_m).destination(center, 90)
    for i in range(res):
        theta = 360 * i / res
        pos = distance(meters=radius_m).destination(right, -theta)
        points.append(GeoPoint(pos.latitude, pos.longitude, center_alt))

    return points

if __name__ == "__main__":
    rospy.init_node("figure_eight_client")
    # rospy.wait_for_service("upload_mission")
    # upload = rospy.ServiceProxy("upload_mission", UploadMission)
    # rospy.sleep(2)
    waypoints = generate_figure_eight(35.681236, 139.767125, 10.0, 2.0)
    print(waypoints)
    # resp = upload(waypoints)
    # print(resp.message)
