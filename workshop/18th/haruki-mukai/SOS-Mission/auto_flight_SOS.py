
from pymavlink import mavutil

# 移動する範囲座標数倍率を指定。
# このスクリプトでは横方向に最大5ブロック、縦方向に最大2ブロック移動する
# ブロック単位での移動座標数を小数点以下7桁で指定しているものに倍率をかけて、それを現在座標から加減算して表現をしている。
# 例えば1ブロック移動でsize = 1000の場合、緯度経度が0.0001度移動することになる。
size = 1000

address = 'udp:127.0.0.1:14550'

is_Sim = False

# Initialize
connection = mavutil.mavlink_connection(address)
# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Initial Position Move
# mode set to guided
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,  # confirmation
    1,  # param1: Mode
    mavutil.mavlink.COPTER_MODE_GUIDED,  # param2: Custom Mode
    0, 0, 0, 0, 0  # param3 - param7: 他のパラメータ
)

if not is_Sim:

    # arm command
    connection.mav.command_long_send(
        connection.target_system,  # target_system
        connection.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
        1,  # confirmation
        1,  # param1 (1 to arm, 0 to disarm)
        0,  # param2 (all other params are ignored)
        0,  # param3
        0,  # param4
        0,  # param5
        0,  # param6
        0   # param7
    )

    # takeoff 10m
    connection.mav.command_long_send(
            connection.target_system,  # Target system ID
            connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # ID of command to send
            0,  # Confirmation
            0,  # param1: Message ID to be streamed
            0, # param2: Interval in microseconds
            0,       # param3 (unused)
            0,       # param4 (unused)
            0,       # param5 (unused)
            0,       # param5 (unused)
            10        # param6 (unused)
            )

    # wait 10m altitude
    while True:
        msg = connection.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            print(msg)
            if msg.relative_alt / 1000 > 10:
                print("Reached 10 meters")
                break


def download_mission(master):
    mission = []
    # ミッションアイテムの数を取得
    master.mav.mission_request_list_send(master.target_system, master.target_component)
    mission_count = master.recv_match(type='MISSION_COUNT', blocking=True).count
    # ミッションアイテムをダウンロード
    for i in range(mission_count):
        master.mav.mission_request_int_send(master.target_system, master.target_component, i)
        item = master.recv_match(type='MISSION_ITEM_INT', blocking=True)
        new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            master.target_system,
            master.target_component,
            item.seq,
            item.frame, item.command, item.current, item.autocontinue,
            item.param1, item.param2, item.param3, item.param4,
            item.x, item.y, item.z,
        )
        mission.append(new_waypoint)
    return mission

def add_waypoint(master,mission, lat, lon, alt):
        print("lat: ", lat , "lon: ", lon, "alt: ", alt)
    # 新しいウェイポイントを追加
    # if mission:
        new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system,#mission[0].target_system,
        master.target_component,
        len(mission),  # シーケンス番号
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        int(lat),
        int(lon),
        int(alt)
        )
        #mission = [new_waypoint]
        mission.append(new_waypoint)
    # else:
    #     print("ミッションが空です。他のウェイポイントを追加する前に、初期のウェイポイントを追加してください。")


def upload_mission(master, mission):
    # ミッションをアップロード
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    master.mav.mission_count_send(master.target_system, master.target_component, len(mission))
    for i, item in enumerate(mission):
        master.mav.send(item)
    # アップロードしたミッションを機体に設定
    master.mav.mission_set_current_send(0, master.target_component, 0)
    master.mav.mission_request_list_send(master.target_system, master.target_component)

# ミッションをクリア
connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)

# ミッションをダウンロード(実質的にはミッションのリスト枠だけを取得)
data = download_mission(connection)

# 現在の位置を取得
response = connection.recv_match(type='GPS_RAW_INT', blocking=True)

# ウェイポイントの作成
def addS(connection,data,response,size,textPosition):
    # S
    add_waypoint(connection,data, response.lat, response.lon + ((textPosition + 0) * size), 30)
    add_waypoint(connection,data, response.lat, response.lon + ((textPosition + 1) * size), 30)
    add_waypoint(connection,data, response.lat, response.lon + ((textPosition + 0) * size), 30)
    add_waypoint(connection,data, response.lat - (1 * size), response.lon + ((textPosition + 0) * size), 20)
    add_waypoint(connection,data, response.lat - (1 * size), response.lon + ((textPosition + 1) * size), 20)
    add_waypoint(connection,data, response.lat - (2 * size), response.lon + ((textPosition + 1) * size), 10)
    add_waypoint(connection,data, response.lat - (2 * size), response.lon + ((textPosition + 0) * size), 10)
    
def addO(connection,data,response,size,textPosition):
    # O
    add_waypoint(connection,data, response.lat, response.lon + ((textPosition + 0) * size), 30)
    add_waypoint(connection,data, response.lat - (2 * size), response.lon + ((textPosition + 0) * size), 10)
    add_waypoint(connection,data, response.lat - (2 * size), response.lon + ((textPosition + 1) * size), 10)
    add_waypoint(connection,data, response.lat, response.lon + ((textPosition + 1) * size), 30)
    add_waypoint(connection,data, response.lat, response.lon + ((textPosition + 0) * size), 30)

addS(connection,data,response,size,0)
addO(connection,data,response,size,2)
addS(connection,data,response,size,4)

# 作成されたミッションのアップロード実行
upload_mission(connection, data)

# ミッションのアップロードが完了するまで待機
while True:
    msg = connection.recv_match(type=['MISSION_ACK'], blocking=True)
    if msg.get_type() == 'MISSION_ACK':
        if msg.type == 0:
            print("Mission uploaded successfully")
        else:
            print("Mission upload failed")
        break

if not is_Sim:
        
    # ミッションの実行
    connection.mav.command_long_send(
        connection.target_system,  # target_system
        connection.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_MISSION_START,  # command
        0,  # confirmation
        0,  # param1 # Hold time. 
        0,  # param2 # Acceptance radius.
        0,  # param3 # Pass through.
        0,  # param4 # Yaw.
        0,  # param5 # Latitude.
        0,  # param6 # Longitude.
        0   # param7 # Altitude.
    )


    # ミッションを全部完遂するまで待機
    while True:
        msg = connection.recv_match(type=['MISSION_ITEM_REACHED'], blocking=True)
        if msg.get_type() == 'MISSION_ITEM_REACHED':
            print("Reached waypoint %d" % msg.seq)
            if msg.seq == len(data) - 1:
                print("All waypoints reached")
                break

    # 帰還
    connection.mav.command_long_send(
        connection.target_system,  # target_system
        connection.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # command
        0,  # confirmation
        0,  # param1 # Hold time. 
        0,  # param2 # Acceptance radius.
        0,  # param3 # Pass through.
        0,  # param4 # Yaw.
        0,  # param5 # Latitude.
        0,  # param6 # Longitude.
        0   # param7 # Altitude.
    )

# 切断
connection.close()