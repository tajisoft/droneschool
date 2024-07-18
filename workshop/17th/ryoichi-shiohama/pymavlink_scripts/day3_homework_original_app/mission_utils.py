from pymavlink import mavutil

def add_waypoint(mission, lat, lon, alt, current=0):
    # 新しいウェイポイントを追加
    new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
        1,
        0,
        len(mission),
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current,
        1, # 1なら自動的に次のウェイポイントに進む
        0,
        0,
        0,
        0,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt)
    )
    mission.append(new_waypoint)


def upload_mission(master, mission):
    # ミッションをアップロード
    master.mav.mission_clear_all_send(
        master.target_system, master.target_component)
    master.mav.mission_count_send(
        master.target_system, master.target_component, len(mission))
    for i, item in enumerate(mission):
        master.mav.send(item)

    # アップロードしたミッションを機体に設定
    master.mav.mission_set_current_send(0, master.target_component, 0)
    master.mav.mission_request_list_send(
        master.target_system, master.target_component)

def create_triangle_mission(master, target_altitude):
    # 現在位置取得
    print("現在位置の取得開始")
    current_position = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_lat = current_position.lat / 1e7
    current_lon = current_position.lon / 1e7
    print("現在位置の取得完了")

    # 三角形の頂点を計算
    distance = 0.0001  # 約11メートル
    wp1_lat = current_lat + distance
    wp1_lon = current_lon

    wp2_lat = current_lat
    wp2_lon = current_lon + distance

    wp3_lat = current_lat - distance
    wp3_lon = current_lon

    # 新しいミッションのリストを作成
    new_mission = []

    # ホームポジションを最初のウェイポイントとして追加
    add_waypoint(new_mission, current_lat, current_lon, target_altitude, current=1)  # 最初のウェイポイント

    # 三角形の頂点をウェイポイントとして追加
    add_waypoint(new_mission, wp1_lat, wp1_lon, target_altitude)
    add_waypoint(new_mission, wp2_lat, wp2_lon, target_altitude)
    add_waypoint(new_mission, wp3_lat, wp3_lon, target_altitude)
    add_waypoint(new_mission, current_lat, current_lon, target_altitude)  # 最後のウェイポイント

    print(f"ミッションの用意完了: {new_mission}")

    # ミッションアップロード
    upload_mission(master, new_mission)
    print("ミッションアップロード完了")
    return new_mission
