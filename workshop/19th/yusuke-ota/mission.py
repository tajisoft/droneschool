from pymavlink import mavutil

def convert_dict_to_mission_items(route_dict, target_sys, target_comp, current_lat, current_lon, current_alt, start_seq=0):
    mission_items = []
    seq = start_seq

    # ダミーWP（seq=0）
    dummy_wp = mavutil.mavlink.MAVLink_mission_item_int_message(
        target_system=target_sys,
        target_component=target_comp,
        seq=seq,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=1,
        autocontinue=1,
        param1=1.0, param2=0, param3=0, param4=0,
        x=int(current_lat * 1e7),
        y=int(current_lon * 1e7),
        z=current_alt
    )
    mission_items.append(dummy_wp)
    seq += 1

    for key in sorted(route_dict, key=lambda k: int(k)):
        wp = route_dict[key]
        lat = int(wp["lat"] * 1e7)
        lon = int(wp["lon"] * 1e7)
        alt = wp["alt"]

        msg = mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=target_sys,
            target_component=target_comp,
            seq=seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0,
            autocontinue=1,
            param1=0,
            param2=30,
            param3=0,
            param4=0,
            x=lat,
            y=lon,
            z=alt
        )
        mission_items.append(msg)
        seq += 1

    return mission_items

def generate_do_jump_item(target_sys, target_comp, seq, jump_to_seq=1, repeat_count=999):
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        target_system=target_sys,
        target_component=target_comp,
        seq=seq,
        frame=mavutil.mavlink.MAV_FRAME_MISSION,
        command=mavutil.mavlink.MAV_CMD_DO_JUMP,
        current=0,
        autocontinue=1,
        param1=jump_to_seq,
        param2=repeat_count,
        param3=0,
        param4=0,
        x=0,
        y=0,
        z=0
    )

def generate_loiter_time_item(target_sys, target_comp, seq, lat=0, lon=0, alt=0, loiter_time=1.0):
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        target_system=target_sys,
        target_component=target_comp,
        seq=seq,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        current=0,
        autocontinue=1,
        param1=loiter_time,
        param2=0, param3=0, param4=0,
        x=int(lat * 1e7),
        y=int(lon * 1e7),
        z=alt
    )


def upload_mission(master, mission_items):
    master.waypoint_clear_all_send()
    master.waypoint_count_send(len(mission_items))
    print(f"Sending {len(mission_items)} mission items...")

    for i, msg in enumerate(mission_items):
        while True:
            req = master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
            if req:
                if req.seq == i:
                    print(f"→ sending item seq={msg.seq}, command={msg.command}")
                    master.mav.send(msg)
                    break
                else:
                    print(f"⚠️ Unexpected seq: expected {i}, got {req.seq}")
            else:
                print("⚠️ Timeout waiting for MISSION_REQUEST")
                return
    print("Mission uploaded successfully.")

def download_mission(master):
    master.waypoint_request_list_send()
    count_msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if count_msg is None:
        print("Timeout waiting for MISSION_COUNT.")
        return []

    total = count_msg.count
    print(f"Mission count: {total}")

    mission_items = []
    for seq in range(total):
        master.mav.mission_request_int_send(master.target_system, master.target_component, seq)
        msg = master.recv_match(type=['MISSION_ITEM_INT', 'MISSION_ITEM'], blocking=True, timeout=5)
        if msg is None:
            print(f"Timeout while receiving mission item {seq}.")
            break
        mission_items.append(msg)

    return mission_items

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', type=str, default='tcp:172.26.176.1:5773')
    args = parser.parse_args()

    print(f"Connecting to {args.connect}...")
    master = mavutil.mavlink_connection(args.connect)
    master.wait_heartbeat()
    print(f"Heartbeat received: sysid={master.target_system}, compid={master.target_component}")

    # 現在位置取得（ダミー用）
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if msg is None:
        raise RuntimeError("Failed to get current position")

    current_lat = msg.lat / 1e7
    current_lon = msg.lon / 1e7
    current_alt = msg.relative_alt / 1000.0

    patrol_route = {
        "1": {"lat": 35.88233960, "lon": 140.34802440, "alt": 100.0},
        "2": {"lat": 35.87343770, "lon": 140.33523560, "alt": 100.0},
        "3": {"lat": 35.86676060, "lon": 140.32356260, "alt": 100.0},
        "4": {"lat": 35.86443050, "lon": 140.31472210, "alt": 100.0},
        "5": {"lat": 35.86450000, "lon": 140.30643940, "alt": 100.0},
        "6": {"lat": 35.86700410, "lon": 140.29807090, "alt": 100.0},
        "7": {"lat": 35.87089910, "lon": 140.29008870, "alt": 100.0},
        "8": {"lat": 35.86700410, "lon": 140.29807090, "alt": 100.0},
        "9": {"lat": 35.86450000, "lon": 140.30643940, "alt": 100.0},
        "10": {"lat": 35.86443050, "lon": 140.31472210, "alt": 100.0},
        "11": {"lat": 35.86676060, "lon": 140.32356260, "alt": 100.0},
        "12": {"lat": 35.87343770, "lon": 140.33523560, "alt": 100.0},
        "13": {"lat": 35.88233960, "lon": 140.34802440, "alt": 100.0}
    }

    mission_items = convert_dict_to_mission_items(patrol_route, master.target_system, master.target_component, current_lat, current_lon, current_alt)
    loiter_time = generate_loiter_time_item(master.target_system, master.target_component, seq=len(mission_items))
    mission_items.append(loiter_time)
    do_jump = generate_do_jump_item(master.target_system, master.target_component, seq=len(mission_items), jump_to_seq=1)
    mission_items.append(do_jump)

    upload_mission(master, mission_items)

    master.mav.mission_set_current_send(master.target_system, master.target_component, 1)
    master.set_mode_auto()

    downloaded = download_mission(master)
    print(f"Uploaded:   {len(mission_items)} items")
    print(f"Downloaded: {len(downloaded)} items")

    for i, (up, down) in enumerate(zip(mission_items, downloaded)):
        if up.command >= 200:
            continue
        if hasattr(down, "x"):
            lat_match = up.x == down.x
            lon_match = up.y == down.y
        else:
            lat_match = int(up.x / 1e7) == int(down.x)
            lon_match = int(up.y / 1e7) == int(down.y)
        if not lat_match or not lon_match:
            print(f"[Mismatch] Seq {i}")
        else:
            print(f"[OK] Seq {i}: command={up.command}")
