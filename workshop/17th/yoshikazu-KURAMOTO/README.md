# Hello, ArduPilot!

## 実行環境
### Flight controller
- JFB110(ardusub4.6.0)
- SITL(arducopter_stable)
### Companion Computer
- Raspberry Pi zero + Waveshare EthernetHAT


## Pymavlink note 
### 1\. pm00_connect_heartbeat.py
- コメント追加
- Mavlinkutil USBシリアル使用
- RaspberryPiGPIOピン使用時の設定追記

### 2\. pm01_message_basics.py
- コメント追加
- Mavlinkutil USBシリアル使用
- RaspberryPiGPIOピン使用時の設定追記
- 実行結果記載

### 3\. pm01_message_dump.py
- mavutil.mavlink_connection udpinを使った設定追加

### 4\. pm02_arm_disarm.py
- master.mav.command_long_sendを使った設定方法追加

### 5\. pm03_change_mode.py
 
- ArdusubではSTABILIZE⇒2 MANUAL⇒0となり、ArduSubのflightmodeの番号と整合しないためUNKNOWN飛行モードとなる
- 何度かやるとSTABILIZE⇒0、MANUAL⇒19となるときがある
- 要調査

### 6\. pm04_change_mode_prompt.py
- ArduSubのモードが認識されたりされず、変更後モードの表示が出ないときがある。
- Mapping がうまく表示されるときとされないときがある。
- 要調査

### 7\. pm10_off.py
- コメント追加
- Arducopter（SITL）での作動確認

### 8\. pm20_read_params.py
- コメント追加
- Arducopter（SITL）での作動確認

### 9\. pm21_set_param.py
- コメント追加
- Arducopter（SITL）での作動確認

### 10\. pm30_home_location.py
- コメント追加
- Arducopter（SITL）での作動確認

### 11\. pm31_home_location_thread.py
- コメント追加
- Arducopter（SITL）での作動確認

### 12\. pm40_location_control.py
- コメント追加
- 飛行モードの表示＆WPまでの残距離も表示するように変更
- Arducopter（SITL）での作動確認

### 13\. pm41_altitude_control.py
- コメント追加
- Arducopter（SITL）での作動確認

### 14\. pm42_position_control.py
- コメント追加
- Arducopter（SITL）での作動確認

### 15\. pm43_command.py
- コメント追加
- Arducopter（SITL）で作動確認

### 16\. pm50_mission_basics.py
- コメント追加
- Arducopter（SITL）で作動確認

## drone-kit code
### 1\. 011_connect.py
- 速度単位追加
- 速度小数点以下第2位まで表示
- Arducopter（SITL）で作動確認

### 2\. 012_attribute.py
- Arducopter（SITL）で作動確認 

### 3\. 013_attribute_util.py
- Arducopter（SITL）で作動確認

### 4\. 014_takeoff.py
- タイムアウトして正常に動かない

### 5\. 015_takeoff.py
- Arducopter(SITL)で確認

## original_app
- 実行すると機体に接続し、離陸高度入力待ちになります。
- 高度を入力するとその高度まで上昇し、上昇中の高度を表示します。
- 離陸高度に到達するとプログラムが終了します。
