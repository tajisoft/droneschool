# 一等無人航空機試験のスクエア飛行のルートを模擬して飛行させる
# WayPintを使ったAUTO飛行ではなくGUIDEDで制御する
#
# flight()は0.2秒周期で呼び出される前提で状態遷移させて制御する
# ※0.2秒の根拠は実験の結果モード変化検出できる最長時間のため
import sys
import time
import signal
from pymavlink import mavutil

flcnt = 0
flstate = 0
lastmode = 'none'
target_alt = 5
staytime = 0
PAI = 3.14159265
TURN180 = PAI
TURN2LEFT = -( PAI / 2 )
tick = 0.2

def intervalReq(master: mavutil.mavfile, msg):
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, msg, 100000, 0, 0, 0, 0, 0)

def delaySec2Cnt(sec):
  global tick
  return sec / tick

def goTurn(master: mavutil.mavfile,rad) :
    # message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 2503 0 0 0 0 0 0 0 0 0 3.14159 0
    # 上記は180度右旋回の場合の設定
    master.mav.set_position_target_local_ned_send(
          0,master.target_system, master.target_component,
          mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
          0b100111000111,0,0,0,0,0,0,0,0,0,rad,0)

def goStraight(master: mavutil.mavfile,dist,alt) :
    print(dist,'m直進')
    # message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3576 5 0 0 0 0 0 0 0 0 0 0
    master.mav.set_position_target_local_ned_send(
          0,master.target_system, master.target_component,
          mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
          0b110111111000,dist,0,alt,0,0,0,0,0,0,0,0)

# 機体への接続（単体実行用：親スクリプトで接続していない時のみ実行）
# ラズベリーパイのGPIOポートのRX/TXとCubeOrangePlusのTELEM2をUART接続
# 配線省略のためにGPSポートのTELEM3を使いたかったがなぜか接続できなかった(追及していない)。
# TELEM1はイームズのテレメトリーユニットと接続している
def setup() -> mavutil.mavfile:
  # master: mavutil.mavfile = mavutil.mavlink_connection(
  #  "/dev/serial0", baud=115200, source_system=1, source_component=90)
  # mavlink-router経由での接続（uart接続はmavlink-routerに任せる）
  master: mavutil.mavfile = mavutil.mavlink_connection(
      "tcp:127.0.0.1:5762", source_system=1, source_component=90)
#      "127.0.0.1:14551", source_system=1, source_component=90)

  master.wait_heartbeat()

  return master

def flight(master: mavutil.mavfile, delay = 0.2):
    global flcnt
    global flstate
    global lastmode
    global target_alt
    global staytime
    global tick
    tick = delay
    try:
      master.recv_match(type='SYS_STATUS',blocking=False)
      nowmode = master.flightmode
      if lastmode != nowmode :
        print(nowmode)
        lastmode = nowmode
    except:
      pass
    if master.flightmode == 'GUIDED' :
      # GUIDEDに切り替わった初期状態
      # GUIDEDへの切り替えはプロポなど外部からの操作で行う
      if flstate == 0 :
        flstate = 1
        print('ACTIVATE GUIDED MODE FLIGHT')
    elif master.flightmode != 'LAND' :
      # 状態初期化
      flstate = 0
    if flstate == 1 :
      # 飛行制御処理開始
      flstate = flstate + 1
      print('FLIGHT START')
    elif flstate == 2 :
      # ARM
      master.arducopter_arm()
      master.motors_armed_wait()
      # ARM確認OK
      flstate = flstate + 1
      print('ARMED')
      intervalReq(master,33)  # GLOBAL_POSITION_INTインターバル要求
    elif flstate == 3 :
      # 離陸（高度5m）
      target_alt = 5
      master.mav.command_long_send(
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
          0, 0, 0, 0, 0, 0, 0, target_alt)
      flstate = flstate + 1
      target_alt = target_alt * 0.95
    elif flstate == 4 :
      # GLOBAL_POSITION_INT から相対高度を取得
      recieved_msg = master.recv_match(
          type='GLOBAL_POSITION_INT', blocking=True)
      current_altitude = recieved_msg.relative_alt / 1000
      if current_altitude >= target_alt:
          # 離陸OK
          flstate = flstate + 1
          staytime = delaySec2Cnt(5)
          print('TAKEOFF 5m')
    elif flstate == 5 :
      # 5秒ホバリング
      if staytime > 0 :
        #print('ホバリング中',staytime)
        staytime = staytime - 1
      else :
        print('ホバリング完了')
        flstate = flstate + 1
    elif flstate == 6 :
      print('180度右旋回',mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED)
      goTurn(master,TURN180)
      flstate = flstate + 1
      staytime = delaySec2Cnt(6)
    elif flstate == 7 :
      # 旋回完了待ち（とりあえず時間で完了判定）
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('180度右旋回完了')
    elif flstate == 8 :
      # 5m直進
      goStraight(master,5,0)
      flstate = flstate + 1
      staytime = delaySec2Cnt(10)
    elif flstate == 9 :
      # A地点到達確認（とりあえず時間で完了判定）
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('A地点到達完了')
    elif flstate == 10 :
      print('左90度旋回')
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
      staytime = delaySec2Cnt(6)
    elif flstate == 11 :
      # 旋回完了待ち（とりあえず時間で完了判定）
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 12 :
      # 6.5m直進
      goStraight(master,6.5,0)
      flstate = flstate + 1
      staytime = delaySec2Cnt(15)
    elif flstate == 13 :
      # B地点到達確認
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('B地点到達完了')
    elif flstate == 14 :
      # 左90度旋回
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
      staytime = delaySec2Cnt(6)
    elif flstate == 15 :
      # 旋回完了待ち（とりあえず時間で完了判定）
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 16 :
      # 10m(現在の高度との差分5m分)に高度を上げながら5m直進
      goStraight(master, 5, -5)
      flstate = flstate + 1
      staytime = delaySec2Cnt(15)
    elif flstate == 17 :
      # C地点到達確認
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('C地点到達完了')
    elif flstate == 18 :
      # 左90度旋回
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
      staytime = delaySec2Cnt(6)
    elif flstate == 19 :
      # 旋回完了待ち（とりあえず時間で完了判定）
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 20 :
      # 13m直進
      goStraight(master,13,0)
      flstate = flstate + 1
      staytime = delaySec2Cnt(25)
    elif flstate == 21 :
      # D地点到達確認
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('D地点到達完了')
    elif flstate == 22 :
      # 左90度旋回
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
      staytime = delaySec2Cnt(6)
    elif flstate ==23 :
      # 旋回完了待ち（とりあえず時間で完了判定）
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 24 :
      # 5mに高度を下げながら5m直進
      goStraight(master, 5, 5)
      flstate = flstate + 1
      staytime = delaySec2Cnt(15)
    elif flstate == 25 :
      # E地点到達確認
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('E地点到達完了')
    elif flstate == 26 :
      # 左90度旋回
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
      staytime = delaySec2Cnt(6)
    elif flstate ==27 :
      # 旋回完了待ち（とりあえず時間で完了判定）
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 28 :
      # 6.5m直進
      goStraight(master, 6.5, 0)
      flstate = flstate + 1
      staytime = delaySec2Cnt(15)
    elif flstate == 29 :
      # A地点到達確認
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('A地点到達完了')
    elif flstate == 30 :
      # 左90度旋回
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
      staytime = delaySec2Cnt(6)
    elif flstate ==31 :
      # 旋回完了待ち（とりあえず時間で完了判定）
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 32 :
      # 5m直進
      goStraight(master, 5, 0)
      flstate = flstate + 1
      staytime = delaySec2Cnt(10)
    elif flstate == 33 :
      # 離陸地点到達確認
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        staytime = delaySec2Cnt(5)
        print('離陸地点到達完了')
    elif flstate == 34 :
      # 5秒ホバリング
      if staytime > 0 :
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('ホバリング完了')
    elif flstate == 35 :
      # 着陸
      master.mav.command_long_send(
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, master.mode_mapping()['LAND'], 0, 0, 0, 0, 0)
      flstate = flstate + 1
      staytime = delaySec2Cnt(10)
    elif flstate == 36 :
      # print('着陸確認',staytime)
      if staytime > 0:
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('着陸完了')

    flcnt = flcnt + 1

if __name__ == "__main__":
    master: mavutil.mavfile = setup()
    print('接続')
    while True:
        #print('before')
        flight(master,0.2)
        #print('after')
        time.sleep(0.2)

