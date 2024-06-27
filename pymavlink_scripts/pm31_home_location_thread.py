import threading
from pymavlink import mavutil


# ホームポジション取得処理
def receive_home_position(master: mavutil.mavfile):
    while True:
        msg = master.recv_match(type="HOME_POSITION", blocking=True)
        if msg is not None:
            home_latitude = msg.latitude / 1.0e7
            home_longitude = msg.longitude / 1.0e7
            home_altitude = msg.altitude / 1000.0

            print(
                f"ホームロケーション: lat={home_latitude}, lon={home_longitude}, alt={home_altitude}"
            )
            break


# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90
)
master.wait_heartbeat()

# メッセージレート変更
msg = master.mav.request_data_stream_encode(
    master.target_system, master.target_component, 0, 10, 1
)
master.mav.send(msg)

# 別スレッドでホームポジション取得処理を呼び出し
thread = threading.Thread(target=receive_home_position, args=(master,))
thread.start()

# 機体のARM
master.arducopter_arm()
print("ARM完了")

# スレッド終了待ち
thread.join()
