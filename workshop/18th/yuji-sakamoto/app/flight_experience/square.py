# 一等無人航空機試験のスクエア飛行のルートを模擬して飛行させる
# flight()は0.2秒周期で呼び出される前提で状態遷移させて制御する
# ※0.2秒の根拠は実験の結果モード変化検出できる最長時間のため
import sys
import time
import signal
from pymavlink import mavutil

flcnt = 0
flstate = 0
lastmode = 'none'

def sendmsg(master: mavutil.mavfile,msg:int):
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, msg, 100000, 0, 0, 0, 0, 0)

def setup() -> mavutil.mavfile:
  # 機体への接続
  master: mavutil.mavfile = mavutil.mavlink_connection(
      "127.0.0.1:14551", source_system=1, source_component=90)
  #
  # ラズベリーパイのGPIOポートのRX/TXとCubeOrangePlusのTELEM2をUART接続
  # 配線省略のためにGPSポートのTELEM3を使いたかったがなぜか接続できなかった(追及していない)。
  # TELEM1はイームズのテレメトリーユニットと接続している
  # master: mavutil.mavfile = mavutil.mavlink_connection(
  #  "/dev/serial0", baud=115200, source_system=1, source_component=90)

  master.wait_heartbeat()

  return master

def flight(master: mavutil.mavfile = setup()):
    global flcnt
    global flstate
    global lastmode
    try:
      master.recv_match(type='SYS_STATUS',blocking=True)
      nowmode = master.flightmode
      if lastmode != nowmode :
        print(nowmode)
        lastmode = nowmode
    except:
      pass
    if master.flightmode != 'GUIDED' :
      # 状態初期化
      flstate = 0
    elif flstate > 0 :
      # 飛行制御処理中
      flstate = 2
    else :
      flstate = 1
      print(master.flightmode)

    flcnt = flcnt + 1

if __name__ == "__main__":
    master: mavutil.mavfile = setup()
    while True:
        flight(master)
        time.sleep(0.2)

