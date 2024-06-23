Hello, ArduPilot!

### Enviroment
## Flight controller
JFB110
## Companion Computer
Raspberry Pi zero + Waveshare EthernetHAT
## firmware
ArduSub 4.6.0

### Code 
## pm00_connect_heartbeat.py
コメント追加
Mavlinkutil USBシリアル使用・RaspberryPiGPIOピン使用時の設定追記

## pm01_message_basics.py
コメント追加
Mavlinkutil USBシリアル使用・RaspberryPiGPIOピン使用時の設定追記
実行結果記載

## pm01_message_dump.py
mavutil.mavlink_connection udpinを使った設定追加

## pm02_arm_disarm.py
master.mav.command_long_sendを使った設定方法追加

## pm03_change_mode.py
STABILIZE⇒2
MANUAL⇒0
となり、ArduSubのflightmodeの番号と整合しないためUNKNOWN飛行モードとなる
何度かやるとSTABILIZE⇒0、MANUAL⇒19となるときがある
要調査

## pm04_change_mode_prompt.py
ArduSubのモードが認識されたりされず、変更後モードの表示が出ないときがある。
要調査
Mapping がうまく表示されるときとされないときがある。

