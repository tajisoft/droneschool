from pymavlink import mavutil
import time

def connect_vehicle():
    master: mavutil.mavfile = mavutil.mavlink_connection(
        # SITLの場合
        "tcp:127.0.0.1:5762",  source_system=1, source_component=90)
        # MissionPlannerの場合
        #"tcp:192.168.3.21:5762",  source_system=1, source_component=90)
    master.wait_heartbeat()
    print("接続完了")
    return master

def change_mode(master, mode):
    # モード変更
    master.set_mode_apm(master.mode_mapping()[mode])

    # モード変更を確認
    while True:
        if master.flightmode == mode:
            break
        master.recv_msg()
    print("変更後モード:", master.flightmode)

def get_current_location(master):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7  # 緯度（度単位）
        lon = msg.lon / 1e7  # 経度（度単位）
        alt = msg.relative_alt / 1000.0  # 相対高度（メートル）
        return lat, lon, alt
    return None

def set_target_location(master, lat, lon, alt):
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # フレーム
        0b110111111000,  # タイプマスク（位置情報のみ送信）
        int(lat * 1e7),  # 緯度（int32形式）
        int(lon * 1e7),  # 経度（int32形式）
        alt,  # 高度（float形式、相対高度）
        0, 0, 0,  # 速度（無視）
        0, 0, 0,  # 加速度（無視）
        0, 0  # ヨー、ヨーレート（無視）
    )

if __name__ == "__main__":
    # 接続
    master = connect_vehicle()
    
    # GUIDEDにモード変更
    change_mode(master, "GUIDED")

    # アーム
    master.arducopter_arm()
    master.motors_armed_wait()
    print("アーム完了")

    # 目標高度(メートル)
    target_altitude = 30

    # 離陸
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude)

    # メッセージレート変更: GLOBAL_POSITION_INT(33)を10Hzで受信
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, 33, 100000, 0, 0, 0, 0, 0)

    # 目標高度への到達を確認
    while True:
        # GLOBAL_POSITION_INT から相対高度を取得
        recieved_msg = master.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = recieved_msg.relative_alt / 1000

        print("上昇高度: {}".format(current_altitude))

        if current_altitude >= target_altitude * 0.95:
            print("目標高度に到達: {}".format(current_altitude))
            break

        time.sleep(0.1)

    # 飛行先 現在地から北へ100m進む
    target_distance = 100

    # 目的地到着判定の許容範囲（緯度・経度の差がこの値以下なら到達とみなす）
    TOLERANCE = 0.00001

    # 現在地を取得
    location = get_current_location(master)
    if location:
        current_lat, current_lon, current_alt = location
        print(f"現在地: Latitude={current_lat}, Longitude={current_lon}, Altitude={current_alt}m")
        
        # 北へ移動する新しい緯度を計算
        new_lat = current_lat + (target_distance / 111000.0)
        new_lon = current_lon  # 経度は変わらない
        
        print(f"目的地: Latitude={new_lat}, Longitude={new_lon}, Altitude={current_alt}m")
        
        # 目的地を設定
        set_target_location(master, new_lat, new_lon, target_altitude)
        
        # 移動の待機
        #time.sleep(10)  # 10秒待機（ドローンが移動する時間）
        while True:
             location = get_current_location(master)
             current_lat, current_lon, current_alt = location
             print(f"現在地: Latitude={current_lat}, Longitude={current_lon}, Altitude={current_alt}m")
             print(f"目的地: Latitude={new_lat}, Longitude={new_lon}, Altitude={current_alt}m")
             
             # 緯度・経度が目標値に到達したかを確認
             lat_diff = abs(new_lat - current_lat)
             lon_diff = abs(new_lon - current_lon)
             print(f"目的地までの差: {lat_diff}, Longitude difference: {lon_diff}\n")
             
             if lat_diff <= TOLERANCE and lon_diff <= TOLERANCE:
                print("目的地に到着\n\n")
                break
    else:
        print("目的地に到着失敗")
    
    # 目標地点に達成したのでLANDに変更して着陸
    change_mode(master, "LAND")

    # 着陸時の高度をログで確認
    last_altitude = 0 # 一つ前の高度を保存
    while True:
        # GLOBAL_POSITION_INT から相対高度を取得
        recieved_msg = master.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = recieved_msg.relative_alt / 1000

        print("着陸時の高度: {}".format(current_altitude))

        """
        # 着陸場所によっては相対高度が0にならないためDISARMEDか相対高度が0.1だと着陸成功としてプログラムをexitする
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            print(armed)
            if not armed or current_altitude <= 0.1:
                print("着陸LAND")
                break
        """

        # 着陸場所によっては相対高度が0mにならないため、
        # 相対高度が1m以下になったら着陸、または高度が前回と同じ場合は着陸したとみなす
        if current_altitude <= 1 or last_altitude == current_altitude:
                print("着陸LAND")
                break
        
        last_altitude = current_altitude
        time.sleep(0.1)

    # 切断
    master.close()
