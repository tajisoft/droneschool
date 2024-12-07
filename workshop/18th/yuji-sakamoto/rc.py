import sys
import time
import signal
import RPi.GPIO as GPIO
from pymavlink import mavutil
from app.flight_experience.square import flight

pinno = 18

def setup() -> mavutil.mavfile:
  # 機体への接続
  # ラズベリーパイとの接続の場合でもmavlink-routerd経由で接続するので直接てきなUART接続不要
  master: mavutil.mavfile = mavutil.mavlink_connection(
      "127.0.0.1:14551", source_system=1, source_component=90)
  #
  # ラズベリーパイのGPIOポートのRX/TXとCubeOrangePlusのTELEM2をUART接続
  # 配線省略のためにGPSポートのTELEM3を使いたかったがなぜか接続できなかった(追及していない)。
  # TELEM1はイームズのテレメトリーユニットと接続している
  # master: mavutil.mavfile = mavutil.mavlink_connection(
  #  "/dev/serial0", baud=115200, source_system=1, source_component=90)

  print('connected') # mavlink-routerdが起動していなければconnectionでブロックされる

  master.wait_heartbeat()

  # 全メッセージを10Hzで受信
  # master.mav.request_data_stream_send(
  #     master.target_system, master.target_component,
  #     0, 10, 1)

  # GLOBAL_POSITION_INT(33)メッセージを10Hzで受信
  # HEARTBEAT (0) 
  #master.mav.command_long_send(
  #  master.target_system, master.target_component,
  #  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
  #  0, 33, 100000, 0, 0, 0, 0, 0)

  # BCM(GPIO番号)で指定する設定
  GPIO.setmode(GPIO.BCM)

  # GPIO18を出力モード設定
  GPIO.setup(pinno, GPIO.OUT)

  return master

def cleanup(master: mavutil.mavfile):
  print('cleanup()')

  # LED消灯
  GPIO.output(pinno, 0)

  # GPIO設定クリア
  GPIO.cleanup()

  # close
  master.close()

def sig_handler(signum, frame) -> None:
    sys.exit(1)

def main():
    cnt = 0
    lcnt = 0
    signal.signal(signal.SIGTERM, sig_handler)
    try:
        master: mavutil.mavfile = setup()
        while True:
            try:
              flight(master)
            except:
              pass
            if( lcnt % 2 ) :
                # LED点灯
                GPIO.output(pinno, 1)
            else :
                # LED消灯
                GPIO.output(pinno, 0)
            cnt = cnt + 1
            if( cnt % 5 ) == 0 :
                lcnt = lcnt + 1
            time.sleep(0.2)

    finally:
        signal.signal(signal.SIGTERM, signal. SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cleanup(master)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

if __name__ == "__main__":
    sys.exit(main())

