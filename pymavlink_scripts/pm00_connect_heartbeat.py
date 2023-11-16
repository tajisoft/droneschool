from pymavlink import mavutil
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5762", source_system=1, source_component=90)
master.wait_heartbeat()

# ターゲットシステムID、コンポーネントIDを表示
print('target_system: {}, target_component: {}'
      .format(master.target_system, master.target_component))

# HEARTBEATメッセージを1秒おきに送信
while True:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0, 0, 0)
    time.sleep(1)
