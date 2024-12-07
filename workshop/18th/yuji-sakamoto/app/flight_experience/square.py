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

def intervalReq(master: mavutil.mavfile,msg:int):
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, msg, 100000, 0, 0, 0, 0, 0)

# 機体への接続（単体実行用：親スクリプトで接続していない時のみ実行）
# ラズベリーパイのGPIOポートのRX/TXとCubeOrangePlusのTELEM2をUART接続
# 配線省略のためにGPSポートのTELEM3を使いたかったがなぜか接続できなかった(追及していない)。
# TELEM1はイームズのテレメトリーユニットと接続している
def setup() -> mavutil.mavfile:
  # master: mavutil.mavfile = mavutil.mavlink_connection(
  #  "/dev/serial0", baud=115200, source_system=1, source_component=90)
  # mavlink-router経由での接続（uart接続はmavlink-routerに任せる）
  master: mavutil.mavfile = mavutil.mavlink_connection(
      "127.0.0.1:14551", source_system=1, source_component=90)

  master.wait_heartbeat()

  return master

def flight(master: mavutil.mavfile):
    global flcnt
    global flstate
    global lastmode
    staytime = 0
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
    elif flstate == 1 :
      # 飛行制御処理開始
      flstate = flstate + 1
      print('FLIGHT START')
    else if flstate == 2 :
      # ARM
      # ARM確認OK
      flstate = flstate + 1
      print('ARMED')
    else if flstate == 3 :
      # 離陸（高度5m）
      # 離陸OK
      flstate = flstate + 1
      staytime = 25 # 5秒（0.2x25）
      print('TAKEOFF 5m')
    else if flstate == 4 :
      # 5秒ホバリング
      if staytime > 0 :
        staytime = staytime - 1
      else :
        flstate = flstate + 1
    else if flstate == 5 :
      # 180度右旋回
      flstate = flstate + 1
    else if flstate == 6 :
      # 5m直進
      flstate = flstate + 1
    else if flstate == 7 :
      # A地点到達確認
      flstate = flstate + 1
    else if flstate == 8 :
      # 左90度旋回
      flstate = flstate + 1
    else if flstate == 9 :
      # 6.5m直進
      flstate = flstate + 1
    else if flstate == 10 :
      # B地点到達確認
      flstate = flstate + 1
    else if flstate == 11 :
      # 左90度旋回
      flstate = flstate + 1
    else if flstate == 12 :
      # 10mに高度を上げながら5m直進
      flstate = flstate + 1
    else if flstate == 13 :
      # C地点到達確認
      flstate = flstate + 1
    else if flstate == 14 :
      # 左90度旋回
      flstate = flstate + 1
    else if flstate == 15 :
      # 13m直進
      flstate = flstate + 1
    else if flstate == 16 :
      # D地点到達確認
      flstate = flstate + 1
    else if flstate == 17 :
      # 左90度旋回
      flstate = flstate + 1
    else if flstate == 18 :
      # 5mに高度を下げながら5m直進
      flstate = flstate + 1
    else if flstate == 19 :
      # E地点到達確認
      flstate = flstate + 1
    else if flstate == 20 :
      # 左90度旋回
      flstate = flstate + 1
    else if flstate == 21 :
      # 6.5m直進
      flstate = flstate + 1
    else if flstate == 22 :
      # A地点到達確認
      flstate = flstate + 1
    else if flstate == 23 :
      # 左90度旋回
      flstate = flstate + 1
    else if flstate == 24 :
      # 5m直進
      flstate = flstate + 1
    else if flstate == 25 :
      # 離陸地点到達確認
      flstate = flstate + 1
      staytime = 25 # 5秒（0.2x25）
    else if flstate == 26 :
      # 5秒ホバリング
      if staytime > 0 :
        staytime = staytime - 1
      else :
        flstate = flstate + 1
    else if flstate == 27 :
      # 着陸
      flstate = flstate + 1
    else if flstate == 28 :
      # DISARM
      flstate = flstate + 1
    else :
      # GUIDEDに切り替わった初期状態
      # GUIDEDへの切り替えはプロポなど外部からの操作で行う
      flstate = 1
      print('ACTIVATE GUIDED MODE FLIGHT')

    flcnt = flcnt + 1

if __name__ == "__main__":
    master: mavutil.mavfile = setup()
    while True:
        flight(master)
        time.sleep(0.2)

