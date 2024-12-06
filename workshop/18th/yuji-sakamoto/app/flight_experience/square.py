# 一等無人航空機試験のスクエア飛行のルートを模擬して飛行させる
# flight()は1秒周期で呼び出される前提で状態遷移させて制御する
import sys
import time
import signal
from pymavlink import mavutil

cnt = 0

def setup() -> mavutil.mavfile:
  # 機体への接続
  #master: mavutil.mavfile = mavutil.mavlink_connection(
  #    "127.0.0.1:14551", source_system=1, source_component=90)
  #
  # ラズベリーパイのGPIOポートのRX/TXとCubeOrangePlusのTELEM2をUART接続
  # 配線省略のためにGPSポートのTELEM3を使いたかったがなぜか接続できなかった(追及していない)。
  # TELEM1はイームズのテレメトリーユニットと接続している
  master: mavutil.mavfile = mavutil.mavlink_connection(
    "/dev/serial0", baud=115200, source_system=1, source_component=90)

  master.wait_heartbeat()

  # 全メッセージを10Hzで受信
  # master.mav.request_data_stream_send(
  #     master.target_system, master.target_component,
  #     0, 10, 1)

  # GLOBAL_POSITION_INT(33)メッセージを10Hzで受信
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0)

  return master

def flight(master: mavutil.mavfile = setup()):
    global cnt
    try:
      print(master.recv_match().to_dict())
    except:
      pass
    cnt = cnt + 1

if __name__ == "__main__":
    master: mavutil.mavfile = setup()
    while True:
        flight(master)
        time.sleep(1)

