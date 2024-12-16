# ドローンエンジニア養成塾１８期坂本作業用

## flight_experience
コース２（アプリケーション）用フライトアプリ課題格納

## syakyo
コース２（アプリケーション）用写経格納

### dronekitサンプル修正
#### 014_takeoff.py
1. GUIDEDモードはスクリプト外で設定するように変更
2. GUIDEDに変更されたら処理を進める
3. GUIDEDに変更されるまでのタイムアウトで処理を終了
4. 目標高度判定修正
5. 離陸後しばらく待ってから着陸するように修正
6. 着陸判定は高度変化がなくなったことを確認する方法で実施

#### 205_parameter_decorator.py
1. 実機接続（CubeOrangePlus(TELEM2)->ラズベリーパイ（mavlink_routerd）->UDP）
2. MPも実機接続してパラメータ値書き換え
3. 通知されることを確認

#### 207_simplegoto.py
1. SITLのロケーションを奥多摩の練習場の座標に設定
2. 目標座標を奥多摩練習場の別の座標を指定
3, 飛行することを確認
4. closeする前に少しsleepして待つように修正

### pymavlinkサンプル修正
#### pm02_arm_disarm.py
1. ARM失敗検出するように修正
2. ARM成功時のみDISARMするように修正

#### pm03_change_mode.py
1. 実機接続（CubeOrangePlus(TELEM2)）->ラズベリーパイ（UART直接）
2. モード変更をスクリプトでは実行せず外部からの変更で検出できるかをお試し
3. master.recv_msg()をmaser.recv_match()に変更してみて動作確認
4. ループ内の待ち時間が長いとモード変更検出できないことを確認

#### pm10_takeoff.py
1. SITL接続に変更
2. ループ時sleep時間をなるべく長めに設定試行錯誤の結果0.2秒に変更
3. 高度変化確認処理修正※本質的な意味は変わらないような修正
4. 離陸後ホバリングして着陸するように修正

#### pm31_home_location_thread.py
1. recv_match()にtimeout設定して完全ブロックしないように修正
2. recv_match()リトライアウトで失敗通知するように修正
3. ARM失敗判定追加
