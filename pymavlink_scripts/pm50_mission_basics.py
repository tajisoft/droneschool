from pymavlink import mavutil

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


def add_waypoint(mission, lat, lon, alt):
    # 新しいウェイポイントを追加
    if mission:
        new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            mission[0].target_system,
            mission[0].target_component,
            len(mission),
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt)
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
        print(element)


if __name__ == '__main__':
    # 機体への接続
    master: mavutil.mavfile = mavutil.mavlink_connection(
        "127.0.0.1:14551", source_system=1, source_component=90)
    master.wait_heartbeat()

    # ミッションのダウンロード
    downloaded_mission = download_mission(master)
    print("ミッションダウンロード完了")
    print_mission(downloaded_mission)

    # ウェイポイントの追加
    add_waypoint(downloaded_mission, lat=35.8792449, lon=140.3394654, alt=5)
    print("ミッションへのウェイポイント追加完了")
    print_mission(downloaded_mission)

    # ウェイポイントの追加
    add_waypoint(downloaded_mission, lat=35.879, lon=140.339, alt=5)
    print("ミッションへのウェイポイント追加完了")
    print_mission(downloaded_mission)

    # アップロード
    upload_mission(master, downloaded_mission)
    print("ミッションアップロード完了")
