# サンプルから以下の点を修正
# 1. ホームポジション取得をブロックさせないようにtimeoutを追加
# 2. 同様にリトライアウトを設定
# 3. ARM要求に対して失敗判定を追加
import threading
from pymavlink import mavutil


# ホームポジション取得処理
limit = 10
def receive_home_position(master: mavutil.mavfile):
    global limit
    while True:
        msg = master.recv_match(type="HOME_POSITION", blocking=True, timeout=5)
        if limit > 0 :
            limit = limit - 1
        elif msg is not None:
            home_latitude = msg.latitude / 1.0e7
            home_longitude = msg.longitude / 1.0e7
            home_altitude = msg.altitude / 1000.0

            print(
                f"ホームロケーション: lat={home_latitude}, lon={home_longitude}, alt={home_altitude}"
            )
            break
        else :
            print("ホームロケーション取得失敗")
            break


# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()

# メッセージレート変更
msg = master.mav.request_data_stream_encode(
    master.target_system, master.target_component, 0, 10, 1)
master.mav.send(msg)

# 別スレッドでホームポジション取得処理を呼び出し
thread = threading.Thread(target=receive_home_position, args=(master,))
thread.start()

# 機体のARM
master.arducopter_arm()
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
  # ARM確認OK
  print("ARM完了")
else :
  print("ARM失敗")


# スレッド終了待ち
thread.join()

master.close()
