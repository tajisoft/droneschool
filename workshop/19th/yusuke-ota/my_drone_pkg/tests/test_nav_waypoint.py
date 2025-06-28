#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division, absolute_import

from pymavlink import mavutil
import time

# 例のWaypoint（必要に応じて変えてください）
waypoints = {
    "0": {"lat": 35.0, "lon": 140.0, "alt": 10},
    "1": {"lat": 35.0001, "lon": 140.0001, "alt": 10},
    "2": {"lat": 35.0002, "lon": 140.0002, "alt": 10}
}

# 接続
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()

# ミッションの全削除（重要）
master.mav.mission_clear_all_send(master.target_system, master.target_component)
time.sleep(1)  # 念のためのインターバル

# MISSION_COUNTの送信
count = len(waypoints)
master.mav.mission_count_send(master.target_system, master.target_component, count)

# リクエストに応じて1つずつ送信
sent_seq = set()
while True:
    msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=5)
    if msg is None:
        print("Timeout waiting for MISSION_REQUEST")
        break

    seq = msg.seq
    if str(seq) not in waypoints:
        print("Invalid sequence:", seq)
        break

    wp = waypoints[str(seq)]
    print("Sending WP seq:", seq)

    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,  # current
        1,  # autocontinue
        0, 0, 0, 0,  # param1-4 (hold time, acceptance radius等。今は未使用)
        wp['lat'],
        wp['lon'],
        wp['alt']
    )

    sent_seq.add(seq)
    if len(sent_seq) == count:
        break

# 最終確認（ACK）
ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
if ack:
    if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission upload accepted.")
    else:
        print("Mission upload failed with type:", ack.type)
else:
    print("No MISSION_ACK received.")
