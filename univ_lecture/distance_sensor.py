import time
import random  # 追加: ランダムな整数を生成するためにrandomモジュールをインポート

# Import mavutil
from pymavlink import mavutil

# UDP接続を確立するためにオートパイロットにpingを送信し、応答を待機する関数
def wait_conn():
    """
    Sends a ping to the autopilot to establish the UDP connection and waits for a reply
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6),  # Unix時間をマイクロ秒単位に変換
            0,  # ピンの番号
            0,  # すべてのシステムへのpingリクエスト
            0  # すべてのコンポーネントへのpingリクエスト
        )
        msg = master.recv_match()
        time.sleep(0.5)

# 接続する
master = mavutil.mavlink_connection('tcp:192.168.1.10:5763', source_system=1, source_component=1)

# 接続を開始するためにpingを送信し、応答を待機する
wait_conn()
print('connected')  # 接続完了を表示する

# オートパイロットをmavlink距離センサーを使用するように設定する。設定を使用するためにオートパイロットを再起動する必要があります。
# master.mav.param_set_send(
#     1,
#     1,
#     b"RNGFND_TYPE",
#     10,  # "MAVLink"
#     mavutil.mavlink.MAV_PARAM_TYPE_INT8)

min_measurement = 10  # オートパイロットが使用する最小有効測定値
max_measurement = 500  # オートパイロットが使用する最大有効測定値
sensor_type = mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER
sensor_id = 1
orientation = mavutil.mavlink.MAV_SENSOR_ROTATION_NONE  # 前向きのセンサー
covariance = 0

tstart = time.time()
while True:
    time.sleep(0.5)
    distance = random.randint(10, 500)  # 10から500のランダムな整数を生成
    master.mav.distance_sensor_send(
        int((time.time() - tstart) * 1000),  # ミリ秒単位の経過時間
        min_measurement,
        max_measurement,
        distance,
        sensor_type,
        sensor_id,
        orientation,
        covariance)
