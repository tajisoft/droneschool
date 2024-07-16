from pymavlink import mavutil
import time

connection_string = 'tcp:127.0.0.1:5762'

#機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    connection_string, source_system=1, source_component=90
)
master.wait_heartbeat()

#ターゲットシステムID、コンポーネントIDを表示
print(f'target_system: {master.target_system}, target_component: {master.target_component}')

#HEATBEATメッセージを1秒おきに送信
while True:
    time.sleep(1)

    # #メッセージ直接送信
    # master.mav.heartbeat_send(
    #     mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    #     mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    #     0, 0, 0
    # )

    #メッセージ作成して送信
    to_send_mgs = master.mav.heartbeat_encode(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0, 0, 0
    )
    master.mav.send(to_send_mgs)