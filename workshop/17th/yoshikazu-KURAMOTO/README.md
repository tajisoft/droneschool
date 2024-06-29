# Hello, ArduPilot!


# 実行環境
### Flight controller
JFB110(ardusub4.6.0_stable) or SITL(arducopter_stable)
### Companion Computer
Raspberry Pi zero + Waveshare EthernetHAT


# Pymavlink note 
1\. pm00_connect_heartbeat.py
コメント追加
Mavlinkutil USBシリアル使用・RaspberryPiGPIOピン使用時の設定追記

2\. pm01_message_basics.py
コメント追加
Mavlinkutil USBシリアル使用・RaspberryPiGPIOピン使用時の設定追記
実行結果記載

3\. pm01_message_dump.py
mavutil.mavlink_connection udpinを使った設定追加

4\. pm02_arm_disarm.py
master.mav.command_long_sendを使った設定方法追加

5\. pm03_change_mode.py
STABILIZE⇒2
MANUAL⇒0
となり、ArduSubのflightmodeの番号と整合しないためUNKNOWN飛行モードとなる
何度かやるとSTABILIZE⇒0、MANUAL⇒19となるときがある
要調査

6\. pm04_change_mode_prompt.py
ArduSubのモードが認識されたりされず、変更後モードの表示が出ないときがある。
要調査
Mapping がうまく表示されるときとされないときがある。

7\. pm10_off.py
コメント追加
Arducopter（SITL）での確認

8\. pm20_read_params.py
コメント追加
Arducopter（SITL）での確認

9\. pm21_set_param.py
コメント追加
Arducopter（SITL）での確認

10\. pm30_home_location.py
コメント追加
Arducopter（SITL）での確認


# drone-kit code
1\.


