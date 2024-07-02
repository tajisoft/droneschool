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
