from pymavlink import mavutil
import numpy as np
import time
import threading

# レーザーセンサーデータをランダムに生成する関数
def generate_laser_scan_data(num_data_points):
    return np.random.uniform(low=1, high=10, size=num_data_points)

# OBSTACLE_DISTANCEメッセージを送信する関数
def send_obstacle_distance_message(master, laser_scan_data):
    N = len(laser_scan_data)
    distances_cm = (laser_scan_data * 100).astype(np.uint16)  # 距離をセンチメートルに変換
    distances = list(distances_cm) + [65535] * (72 - len(distances_cm))  # 未使用の部分を65535で埋める
    time_usec = int(time.time() * 1e3)  # 現在のUNIX時間をミリ秒に変換
    sensor_type = mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER
    increment = int(90 / N)
    min_distance = 1  # 単位はセンチメートル
    max_distance = 500  # 単位はセンチメートル
    increment_f = increment
    angle_offset = -45
    frame = mavutil.mavlink.MAV_FRAME_BODY_FRD # 12
    master.mav.obstacle_distance_send(
        time_usec,
        sensor_type,
        distances,
        increment,
        min_distance,
        max_distance,
        increment_f,
        angle_offset,
        frame
    )

# HEARTBEATメッセージを1秒ごとに送信する関数
def send_heartbeat(master):
    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0,
            0,
            0
        )
        time.sleep(1)

def main():
    master = mavutil.mavlink_connection('tcp:192.168.1.10:5763', source_system=1, source_component=93)
    print("Waiting for heart beat...")
    master.wait_heartbeat()
    print("Heartbeat received")

    # HEARTBEATメッセージを1秒ごとに別スレッドで送信開始
    threading.Thread(target=send_heartbeat, args=(master,)).start()

    while True:
        laser_scan_data = generate_laser_scan_data(72)
        send_obstacle_distance_message(master, laser_scan_data)
        time.sleep(1/15)  # 15Hzになるようにスリープ

if __name__ == "__main__":
    main()
