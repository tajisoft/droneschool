# Hello, ArduPilot!
# ドローンエンジニア養成塾１８期坂本作業用

## app
コース２（アプリケーション）用課題格納

## fc
コース３（フライトコード）用課題格納

## カレントフォルダ
ラズベリーパイでの実験用

### pm01_message_dump.py
gpio(uart)@ラズベリーパイとTELEM2@CubeOrangePlusと接続するお試し
併せてsignalキャッチ時の処理対応

### rc.local
ラズベリーパイにリモート接続できない前提で起動時に自動実行するためのシェルスクリプト
/etc/rc.localに配置する

### rc.py
1. rpanionサーバでhttp://10.0.2.100:3000/controllerで[Start Telemetry]を実行するとmavlink-routerdが動作し、[Stop Telemetry]するとdaemonは停止する機能を利用する

2. mavlink-routerdでラズベリーパイとCubeOrangePlusのTELEM2とシリアル接続させて、mavlink-routerd経由でrc.py側でUDP接続するようにする

3. LED接続して「Lチカ」によりスクリプト動作が確認できるようにする
4. メッセージ受信をblockさせると停止しているように見えるのでblockさせないようなスクリプト構造とする
5. rc.pyはGUIDEDモード時に自動飛行処理させるためのスクリプトを呼び出すが、
ラズベリーパイ上のファイル配置としてdroneschoolリポジトリの個人のフォルダを
参照するようにする
