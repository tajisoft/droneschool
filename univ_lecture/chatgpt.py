import time
import random
import threading
from pymavlink import mavutil

# 接続先の設定
master = mavutil.mavlink_connection('tcp:192.168.10.10:5763', source_system=1, source_component=93)

# 距離情報を保存する変数
distance = 0

# 継続フラグ
running = True

# メインスレッドでdistanceの値をランダムに2Hzで更新
def update_distance():
    global distance
    global running
    while running:
        distance = random.randint(1, 100)
        time.sleep(0.5)  # 2Hz

# 送信スレッドでDISTANCE_SENSORを10Hzで送信
def send_distance_sensor():
    global distance
    global running
    while running:
        master.mav.distance_sensor_send(
            time_boot_ms=int(time.time() * 1000),
            min_distance=1,
            max_distance=100,
            current_distance=distance,
            type=0,
            id=0,
            orientation=0,
            covariance=0,
        )
        time.sleep(0.1)  # 10Hz

# スレッドの開始
update_thread = threading.Thread(target=update_distance)
send_thread = threading.Thread(target=send_distance_sensor)
update_thread.start()
send_thread.start()

try:
    # スレッドの終了待ち
    update_thread.join()
    send_thread.join()
except KeyboardInterrupt:
    # Ctrl+Cが押されたときはフラグを落としてスレッドを停止
    running = False
    update_thread.join()
    send_thread.join()
