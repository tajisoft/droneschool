from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
        "tcp:127.0.0.1:5762", source_system=1, source_component=90)
#    "127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()

master.arducopter_arm()
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
  # ARM確認OK
  print('ARMED')

  time.sleep(5)

  master.arducopter_disarm()
  master.motors_disarmed_wait()
  print("DISARMED")
else :
  # 事前にrc 3 1500などを実行しておくと失敗する
  print('ARM FAILED')
#master.motors_armed_wait()
#print("ARMED")
