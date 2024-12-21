# 一等無人航空機試験のスクエア飛行のルートを模擬して飛行させる
# WayPintを使ったAUTO飛行ではなくGUIDEDで制御する
#
# flight()は0.2秒周期で呼び出される前提で状態遷移させて制御する
# ※0.2秒の根拠は実験の結果モード変化検出できる最長時間のため
#
# 以下を参考に処理する
# https://mavlink.io/en/mavgen_python/howto_requestmessages.html
# https://mavlink.io/en/messages/common.html
# https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
# https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
#
import sys
import time
import signal
from pymavlink import mavutil

flcnt = 0
flstate = 0
lastmode = 'none'
target_alt = 5
staytime = 0
retrytime = 0
PAI = 3.14159265
TURN180 = PAI
TURN2LEFT = -( PAI / 2 )
INT_DISABLE = -1
stableCnt = 0
lastYaw = 0
lastAlt = 0
stableCheck = 2
isActive = False
VPASS = 30
YAWPASS = 20

# 状態遷移制御する場合の特別な状態番号のみ名前を付ける
# ※enumを使うと行数を浪費するので省略
STATE_LAND = 35
STATE_INVALID = 37

tick = 0.2

# FC reboot
def reboot(master: mavutil.mavfile):
    master.mav.command_long_send(master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0)

# 高度が不正かどうかを判定する
# 本当はARMEDとかPREARM可能かなどもチェックしたいが・・・
def isInvalidFly(master: mavutil.mavfile) :
  recv = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
  if recv.relative_alt > 1000 and flcnt > 10 :
    print("recv.relative_alt :", recv.relative_alt, flcnt)
    # recv = master.recv_match(type='RC_CHANNELS_SCALED', blocking=True)
    # print(recv.chan1_scaled,recv.chan2_scaled,recv.chan3_scaled,recv.chan4_scaled)
    return True
  else :
    return False

def isPreArmOk(master: mavutil.mavfile) :
  return True

def isStable(master: mavutil.mavfile) :
  global stableCnt
  global lastYaw
  global lastAlt
  global staytime
  global retrytime
  global stableCheck
  recv = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
  diffYaw = lastYaw - recv.hdg
  diffAlt = lastAlt - recv.relative_alt
  lastYaw = recv.hdg
  lastAlt = recv.relative_alt
  # 旋回や直進指示に対して動き出すのにタイムラグがあるので最低限時間で待ってから判定する
  if staytime > 0 :
    staytime = staytime - 1
    stableCnt = stableCheck
    return False
  # x/y/z方向の速度が0になり機首角度が安定して高度も安定したかどうかを判定する
  elif ( recv.vx > -VPASS and recv.vx < VPASS and
         recv.vy > -VPASS and recv.vy < VPASS and
         recv.vz > -VPASS and recv.vz < VPASS and
         diffAlt > -VPASS and diffAlt < VPASS and
         diffYaw > -YAWPASS and diffYaw < YAWPASS ) :
    stableCnt = stableCnt - 1
    if stableCnt <= 0 :
      # 規定回数安定判断
      return True
    else :
      return False
  elif retrytime > 0 :
    retrytime = retrytime - 1
    print('不安定',recv.vx,recv.vy,recv.vz,recv.relative_alt,recv.hdg,stableCnt)
    stableCnt = stableCheck
    return False
  else :
    print('不安定リトライアウト',recv.vx,recv.vy,recv.vz,recv.relative_alt,recv.hdg,stableCnt)
    return True

def intervalReq(master: mavutil.mavfile, intsec=0.1, msgid=mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
  if intsec < 0 :
    intusec = INT_DISABLE # desable
  else :
    intusec = intsec * 100000
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, msgid, intusec, 0, 0, 0, 0, 0)

def delaySec2Cnt(sec):
  global tick
  global staytime
  global retrytime
  staytime =  sec / tick
  retrytime = staytime * 1.5

def goTurn(master: mavutil.mavfile,rad) :
    # message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 2503 0 0 0 0 0 0 0 0 0 3.14159 0
    # 上記は180度右旋回の場合の設定例
    master.mav.set_position_target_local_ned_send(
          0,master.target_system, master.target_component,
          mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
          0b100111000111,0,0,0,0,0,0,0,0,0,rad,0)
    if rad < 0 :
      rad = -rad
    delaySec2Cnt(rad * 4 / PAI)

def goStraight(master: mavutil.mavfile,dist,alt) :
    print(dist,'m直進')
    # message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3576 5 0 0 0 0 0 0 0 0 0 0
    master.mav.set_position_target_local_ned_send(
          0,master.target_system, master.target_component,
          mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
          0b110111111000,dist,0,alt,0,0,0,0,0,0,0,0)
    if alt > 0 :
      # 降下しながら直進の場合は完了するまで遅い
      delaySec2Cnt(dist)
    else :
      # 高度維持または上昇しながら直進
      delaySec2Cnt(dist/2)

# 機体への接続（単体実行用：親スクリプトで接続していない時実行）
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
    global staytime
    global tick
    global isActive
    tick = delay
    try:
      #print('recv_match()',flcnt,flstate)
      recv = master.recv_match(type='HEARTBEAT', blocking=True)
      if recv.system_status == mavutil.mavlink.MAV_STATE_ACTIVE :
        isActive = True
      elif recv.system_status == mavutil.mavlink.MAV_STATE_STANDBY :
        isActive = False
      #master.recv_match(type='SYS_STATUS',blocking=False)
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
      print('FLIGHT CONTROL START')
      intervalReq(master)  # GLOBAL_POSITION_INTインターバル要求
    elif flstate == 2 :
      # ARM
      if isActive :
        # GUIDEDに切り替えた時に既に飛行していたら無効にする(HEARTBEATでステータスチェック)
        flstate = STATE_INVALID
        isActive = False
        print('FLIGHT CONTROL CANCEL')
      elif isInvalidFly(master) :
        # GUIDEDに切り替えたときに不正に高度の値がゼロ付近以外
        flstate = STATE_INVALID
        print('FLIGHT CONTROL INVALID => reboot')
        reboot(master) # FCをrebootさせる
      elif isPreArmOk(master) :
        master.arducopter_arm()
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
          # ARM確認OK
          flstate = flstate + 1
          print('ARMED')
        else :
          print('ARM FAILED')
          flstate = STATE_INVALID #無効にする 
        #master.motors_armed_wait()
    elif flstate == 3 :
      # 離陸（高度5m）
      master.mav.command_long_send(
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
          0, 0, 0, 0, 0, 0, 0, target_alt)
      flstate = flstate + 1
      delaySec2Cnt(5)
    elif flstate == 4 :
      # 離陸完了待ち
      if isStable(master) :
          # 離陸OK
          flstate = flstate + 1
          delaySec2Cnt(5)
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
      print('180度右旋回')
      goTurn(master,TURN180)
      flstate = flstate + 1
    elif flstate == 7 :
      # 旋回完了待ち
      if isStable(master) :
        flstate = flstate + 1
        print('180度右旋回完了')
    elif flstate == 8 :
      # 5m直進
      goStraight(master,5,0)
      flstate = flstate + 1
    elif flstate == 9 :
      # A地点到達確認
      if isStable(master):
        flstate = flstate + 1
        print('A地点到達完了')
    elif flstate == 10 :
      print('左90度旋回')
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
    elif flstate == 11 :
      # 旋回完了待ち
      if isStable(master) :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 12 :
      # 6.5m直進
      goStraight(master,6.5,0)
      flstate = flstate + 1
    elif flstate == 13 :
      # B地点到達確認
      if isStable(master) :
        flstate = flstate + 1
        print('B地点到達完了')
    elif flstate == 14 :
      print('左90度旋回')
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
    elif flstate == 15 :
      # 旋回完了待ち
      if isStable(master) :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 16 :
      print('10m(現在の高度との差分5m分)に高度を上げながら5m直進')
      goStraight(master, 5, -5)
      flstate = flstate + 1
    elif flstate == 17 :
      # C地点到達確認
      if isStable(master) :
        flstate = flstate + 1
        print('C地点到達完了')
    elif flstate == 18 :
      print('左90度旋回')
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
    elif flstate == 19 :
      # 旋回完了待ち
      if isStable(master) :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 20 :
      print('13m直進')
      goStraight(master,13,0)
      flstate = flstate + 1
    elif flstate == 21 :
      # D地点到達確認
      if isStable(master) :
        flstate = flstate + 1
        print('D地点到達完了')
    elif flstate == 22 :
      print('左90度旋回')
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
    elif flstate ==23 :
      # 旋回完了待ち
      if isStable(master) :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 24 :
      print('5mに高度を下げながら5m直進')
      goStraight(master, 5, 5)
      flstate = flstate + 1
    elif flstate == 25 :
      # E地点到達確認
      if isStable(master) :
        flstate = flstate + 1
        print('E地点到達完了')
    elif flstate == 26 :
      print('左90度旋回')
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
    elif flstate ==27 :
      # 旋回完了待ち
      if isStable(master) :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 28 :
      print('6.5m直進')
      goStraight(master, 6.5, 0)
      flstate = flstate + 1
    elif flstate == 29 :
      # A地点到達確認
      if isStable(master) :
        flstate = flstate + 1
        print('A地点到達完了')
    elif flstate == 30 :
      print('左90度旋回')
      goTurn(master,TURN2LEFT)
      flstate = flstate + 1
    elif flstate ==31 :
      # 旋回完了待ち
      if isStable(master) :
        flstate = flstate + 1
        print('90度左旋回完了')
    elif flstate == 32 :
      print('5m直進')
      goStraight(master, 5, 0)
      flstate = flstate + 1
    elif flstate == 33 :
      # 離陸地点到達確認
      if isStable(master) :
        flstate = flstate + 1
        delaySec2Cnt(5)
        print('離陸地点到達完了')
    elif flstate == 34 :
      # 5秒ホバリング
      if staytime > 0 :
        staytime = staytime - 1
      else :
        flstate = flstate + 1
        print('ホバリング完了')
    elif flstate == STATE_LAND :
      # 着陸
      master.mav.command_long_send(
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, master.mode_mapping()['LAND'], 0, 0, 0, 0, 0)
      flstate = flstate + 1
      delaySec2Cnt(10)
    elif flstate == 36 :
      # print('着陸確認',staytime)
      if isStable(master) :
        flstate = flstate + 1
        print('着陸完了')
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        intervalReq(master, INT_DISABLE)  # GLOBAL_POSITION_INTインターバル停止要求

    flcnt = flcnt + 1

if __name__ == "__main__":
    master: mavutil.mavfile = setup()
    print('接続')
    intervalReq(master, INT_DISABLE)  # GLOBAL_POSITION_INTインターバル停止要求
    while True:
        #print('before')
        flight(master,0.2)
        #print('after')
        time.sleep(0.2)

