from pymavlink import mavutil
import time

# 機体への接続
# Raspberry pi 6pin-GND 8pin-UART_TXD 10pin-UART_RXDで接続
# Raspberry pi /dev/ttyAMA0 baudrate 115200 ⇒　FCとRaspberry piで違う電源を使用しているためGNDレベルあわずダメ?
# Raspberry pi USB FTDI 接続　/dev/ttyUSB0 baudrate 115200　⇒　接続確認できた
# FC-rasPiとするとシリアルがバッティングするので、mavlinkrouterでデータの橋渡しをする

master: mavutil.mavfile = mavutil.mavlink_connection(
    
    # connect vehicle via GPIO 6,8,10pin
    #"/dev/ttyAMA0", baudrate=115200, source_system=1, source_component=90
    
    # connect vehicle via USB serial device
    #"/dev/ttyUSB0", baudrate=115200, source_system=1, source_component=90

    # connect vehicke via udp
    "127.0.0.1:14551", source_system=1, source_component=90
    #'udpin:localhost:14551', source_system=1, source_component=90

    # SITL connected to the vehicle via TCP
    # tcp:127.0.0.1:5760

)
master.wait_heartbeat()

#　ターゲットシステムID、コンポーネントIDを表示
print(f"target_system: {master.target_system}, target_component: {master.target_component}")

while True:
    time.sleep(1)

    # メッセージを直接送信
    #master.mav.heartbeat_send(
    #    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    #    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    #    0,0,0
    #)

    # メッセージ作製して送信
    to_send_msg = master.mav.heartbeat_encode(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0,0,0
    )

