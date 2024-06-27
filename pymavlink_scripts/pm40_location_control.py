import time
from pymavlink import mavutil

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    device="127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()

# GUIDEDモードに設定
master.set_mode(4)

# 目標の緯度・経度、高度を設定
target_lat = 35.8787276
target_lon = 140.3388137
target_alt = 3

# SET_POSITION_TARGET_GLOBAL_INTメッセージを送信
master.mav.set_position_target_global_int_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # グローバル座標、相対高度
    0b0000111111111000,
    int(target_lat * 1e7), int(target_lon * 1e7), target_alt,
    0, 0, 0, 0, 0, 0, 0, 0, 0,)

# 位置情報をログに出し続ける
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000.0
        print(f"Current position: lat={current_lat}, lon={current_lon}, alt={current_alt}")

        # 目的地に近いかどうかを判定
        if abs(current_lat - target_lat) < 0.0001 and abs(current_lon - target_lon) < 0.0001 and abs(current_alt - target_alt) < 0.5:
            print("Approaching target position, stopping logging.")
            break
            # stop_event.set()

    time.sleep(1)
    
print("Position logging stopped. Reached target position.")
