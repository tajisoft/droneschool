from pymavlink import mavutil
import time

# 機体（シミュレータ）への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
  "tcp:10.0.2.135:5762",
  source_system=1,
  source_component=90
)
master.wait_heartbeat()
print("接続完了")

# GUIDEDモードに変更
mode = 'GUIDED'
master.set_mode_apm(master.mode_mapping()[mode])

# モード変更を確認
while True:
  if master.flightmode == mode:
    break
  master.recv_msg()
print("モード変更完了")
