from pymavlink import mavutil
import time

def get_current_location(master):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7  # 緯度（度単位）
        lon = msg.lon / 1e7  # 経度（度単位）
        alt = msg.relative_alt / 1000.0  # 相対高度（メートル）
        return lat, lon, alt
    return None

if __name__ == "__main__":
    master: mavutil.mavfile = mavutil.mavlink_connection(
            # SITLの場合
            #"tcp:127.0.0.1:5762",  source_system=1, source_component=90)
            # MissionPlannerの場合
            # --home 35.867003,140.305987,7,0
            "tcp:192.168.3.21:5762",  source_system=1, source_component=90)
    master.wait_heartbeat()
    print("接続完了")

    speed=100.0

    # ARM the master
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confir]mation
        1,  # ARM
        0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    time.sleep(2)

    # モード変更
    mode = "GUIDED"
    master.set_mode_apm(master.mode_mapping()[mode])

    # モード変更を確認
    while True:
        if master.flightmode == mode:
            break
        master.recv_msg()
    print("変更後モード:", master.flightmode)

    # 各拠点のGPS情報
    # 隣接ポイント
    start_lat = 35.867003
    start_lon = 140.305987
    # セブンイレブン
    seveneleven_lat = 35.877518
    seveneleven_lon = 140.295439
    # 高度
    alt=7

    # どっちの方向に向いて飛んでいるか
    direction = 1 # 0:隣接ポート 1:セブンイレブン 

    # セブンイレブンをまずターゲットにする
    target_lat = seveneleven_lat
    target_lon = seveneleven_lon

    # ターゲットまで走る
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # フレーム
        0b110111111000,  # タイプマスク（位置情報のみ送信）
        int(target_lat * 1e7),  # 緯度（int32形式）
        int(target_lon * 1e7),  # 経度（int32形式）
        alt,  # 高度（float形式、相対高度）
        10, 0, 0,  # 速度（無視）
        0, 0, 0,  # 加速度（無視）
        0, 0  # ヨー、ヨーレート（無視）
    )

    # 目的地到着判定の許容範囲（緯度・経度の差がこの値以下なら到達とみなす）
    TOLERANCE = 0.0001

    # セブンイレブンと隣接ポイント間の走行
    while True:
        location = get_current_location(master)
        current_lat, current_lon, current_alt = location
        print("direction: ",direction)
        print(f"現在地: Latitude={current_lat}, Longitude={current_lon}, Altitude={current_alt}m")
        print(f"目的地: Latitude={target_lat}, Longitude={target_lon}, Altitude={current_alt}m")
        
        # 緯度・経度が目標値に到達したかを確認
        lat_diff = abs(target_lat - current_lat)
        lon_diff = abs(target_lon - current_lon)
        print(f"目的地までの差: {lat_diff}, Longitude difference: {lon_diff}\n")
        
        if lat_diff <= TOLERANCE and lon_diff <= TOLERANCE:
            print("目的地に到着\n\n")

            time.sleep(2)

            # セブンイレブンについたら隣接ポイントに引き返す
            # 隣接ポイントについたらセブンイレブンに向かう
            # これを繰り返す
            match direction:
                case 0:
                    direction = 1
                    target_lat = seveneleven_lat
                    target_lon = seveneleven_lon
                case 1:
                    direction = 0
                    target_lat = start_lat
                    target_lon = start_lon  

            # ターゲットまで走る
            master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms (not used)
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # フレーム
                0b110111111000,  # タイプマスク（位置情報のみ送信）
                int(target_lat * 1e7),  # 緯度（int32形式）
                int(target_lon * 1e7),  # 経度（int32形式）
                alt,  # 高度（float形式、相対高度）
                10, 0, 0,  # 速度（無視）
                0, 0, 0,  # 加速度（無視）
                0, 0  # ヨー、ヨーレート（無視）
            )

        # Stop
        #master.mav.manual_control_send(master.target_system, 0, 0, 0, 0, 0)
        #print("Stopping rover")