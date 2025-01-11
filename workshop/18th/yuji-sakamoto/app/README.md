# ドローンエンジニア養成塾１８期坂本作業用

## flight_experience
コース２（アプリケーション）フライトエクスペリエンス用スクリプト格納

## /etc/rc.local -> rc.pyから呼び出されるスクリプト
### square.py
#### 処理概要
1. 一等無人航空機試験の「高度変化を伴うスクエア飛行」のルートを模擬して飛行させる
2. WayPintを使ったAUTO飛行ではなくGUIDEDで制御する
3. flight()は0.2秒周期で呼び出される前提で状態遷移させて制御する※0.2秒の根拠は実験の結果モード変化検出できる最長時間のため
4. 離陸して目標高度に達した時の判定、旋回完了判定、直進完了判定は共通関数isStable()で実施
5. isStable()はGLOBAL_POSITION_INTメッセージのvx/vy/vzによる速度が0付近になったこととrelative_altの変化がなくなったこと、および、hdgによるヨーの変化がなくなったことによる判定を行っている
6. isStable()による「安定判定」を厳しくするとなかなか安定判定とならないため実機による実験で緩めの設定に変更
7. 強風対策のためisStable()の「安定判定」は時間軸によるリトライアウトを設ける
8. 強風対策のため機首の向きが変わらないように安定判定中とホバリング中は周期的にYAWを補正するように制御する
9. 参考サイト：https://mavlink.io/en/mavgen_python/howto_requestmessages.html
10. 参考サイト：https://mavlink.io/en/messages/common.html
11. 参考サイト：https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
12. 実機テスト動画：https://www.instagram.com/p/DDq2LVBSSes/
13. 強風対策後実機テスト動画：https://www.instagram.com/p/DEC0mtiSUyR/
14. 強風対策後強風時実機テスト動画：https://www.instagram.com/p/DEbDB-nzPCr/

#### 動作環境パッケージインストール手順
1. Windows環境でMission Plannerセットアップ：https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msiをインストール
2. Ubuntu-22.04@WSLをインストール（手順割愛）
3. Visual Studio Codeのインストールと環境設定は動作確認自体には不要
4. ArduPilotビルド環境セットアップ１（Ubuntu@WSLターミナルの特定のフォルダで「git clone https://github.com/ArduPilot/ardupilot.git」実行）
5. ArduPilotビルド環境セットアップ２（Ubuntu@WSLフォルターミナルでgit cloneしたフォルダで「./Tools/environment_install/install-prereqs-ubuntu.sh -y」実行）
6. 5.で.bashrcの修正とPATH設定が行われるので新しいUbuntu@WSLターミナルを開くかターミナルを開きなおす
7. SITLセットアップ（Ubuntu@WSLターミナルでSITL実行環境用のフォルダを作成（そのフォルダにログなどが出力されるため~/SITLなど任意））
8. 7.のフォルダに移動して「sim_vehicle.py -v Copter --map --console -L Kawachi」実行
9. SITL起動したUbuntu@WSLとは別のターミナルを開く
10. square.py取得（9.のUbuntu@WSLターミナルの任意のフォルダで「wget https://raw.githubusercontent.com/pff01210/droneschool/refs/heads/18th_yuji-sakamoto/workshop/18th/yuji-sakamoto/app/flight_experience/square.py」実行）
11. 10.のフォルダで「$ python square.py」実行でSITLと接続して実行開始可能

#### テスト手順（SITL）
1. Ubuntu@WSLターミナルを開き（ターミナル１）SITLを起動
2. 別のUbuntu@WSLターミナルを開き（ターミナル２）square.pyを保存したフォルダで「python square.py」実行
3. Windows上でMP起動してターミナル１のSITLと接続（動作モニター用）
4. ターミナル１のSITLよりコマンド入力(mode stabilize実行後mode guided実行)するとスクエア飛行実行開始される
5. 4.実行中、ターミナル２ではログにより動作状況が出力されている
6. 4.実行中、MPでは動作状況が表示される（移動距離が少ないのでマップはズームインで確認推奨）


#### テスト手順（実機）※参考
0. http://10.0.2.100:3000/controllerで[Start Telemetry]しておく
1. ラズベリーパイのrc.local->rc.py->square.pyを実行
2. rc.pyではfrom app.flight_experience.square import flight
3. rc.pyでmavlink-routerdでFCとUART接続しているのでUDP転送ポートに接続してflight()を0.2秒周期で呼び出す
4. プロポでstabilize->guidedに切り替える
5. スクエア飛行実行
6. guided切り替え時に高度情報が異常の場合は警告音後実行しない
7. guided切り替え後にARMできない場合は警告音後実行しない

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
