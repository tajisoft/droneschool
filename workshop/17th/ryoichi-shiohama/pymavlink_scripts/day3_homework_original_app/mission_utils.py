from pymavlink import mavutil
from math import radians, sin, cos, sqrt, atan2

def download_mission(master):
    mission = []

    # ミッションアイテムの数を取得
    master.mav.mission_request_list_send(
        master.target_system, master.target_component)
    mission_count = master.recv_match(
        type='MISSION_COUNT', blocking=True).count

    # ミッションアイテムをダウンロード
    for i in range(mission_count):
        master.mav.mission_request_int_send(
            master.target_system, master.target_component, i)
        item = master.recv_match(type='MISSION_ITEM_INT', blocking=True)
        new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            master.target_system,
            master.target_component,
            item.seq,
            item.frame,
            item.command,
            item.current,
            item.autocontinue,
            item.param1, item.param2, item.param3, item.param4,
            item.x, item.y, item.z,
        )
        mission.append(new_waypoint)

    return mission

def add_waypoint(mission, lat, lon, alt, current=0):
    # 新しいウェイポイントを追加
    if mission:
        new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            mission[0].target_system,
            mission[0].target_component,
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
            int(alt) # TODO: *1000が必要か検討する
        )
        mission.append(new_waypoint)
    else:
        print("ミッションが空です。他のウェイポイントを追加する前に、初期のウェイポイントを追加してください。")


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


def print_mission(mission):
    for element in mission:
        print(f"e: {element}")


def create_mission(master, target_altitude):
    # ミッションのダウンロード
    downloaded_mission = download_mission(master)
    print(f"ミッションダウンロード完了: {downloaded_mission}")
    print_mission(downloaded_mission)

    # 現在位置取得
    print("現在位置の取得開始")
    current_position = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    home_lat = current_position.lat / 1e7
    home_lon = current_position.lon / 1e7
    print("現在位置の取得完了")

    # 三角形の頂点を計算
    distance = 0.0001  # 約11メートル
    wp1_lat = home_lat + distance
    wp1_lon = home_lon

    wp2_lat = home_lat
    wp2_lon = home_lon + distance

    wp3_lat = home_lat - distance
    wp3_lon = home_lon

    # ホームポジションを最初のウェイポイントとして追加
    add_waypoint(downloaded_mission, home_lat, home_lon, target_altitude, current=1)  # 最初のウェイポイント

    # 三角形の頂点をウェイポイントとして追加
    add_waypoint(downloaded_mission, wp1_lat, wp1_lon, target_altitude)
    add_waypoint(downloaded_mission, wp2_lat, wp2_lon, target_altitude)
    add_waypoint(downloaded_mission, wp3_lat, wp3_lon, target_altitude)
    add_waypoint(downloaded_mission, home_lat, home_lon, target_altitude)  # 最後のウェイポイント

    # ミッションアップロード
    upload_mission(master, downloaded_mission)
    print("ミッションアップロード完了")
    return downloaded_mission


# ホームロケーションとの距離を計算する
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # 地球の半径（メートル）
    dlat = radians(lat2 - lat1)
    dlon = radians(lon1 - lon2)
    a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) * sin(dlon / 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance
