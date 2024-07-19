# 17期宿題内容

## Day3宿題（写経）

### Pymavlink
- pm00_connect_heartbeat.py
- pm10_takeoff.py
- pm30_home_location.py（デバッグ目的でアレンジしました）

### DroneKit
- 011_connect.py
- 201_listner.py

## Day4宿題（終了課題）

「アーム > 離陸 > 自動航行（三角軌道） > RTL」のオリジナルアプリを作成しました。

- day3_homework_original_app/original_mission.py: 一連の処理（エントリーポイント。実行ファイル）
- day3_homework_original_app/mission_utils.py: 三角軌道のミッション作成＆アップロード＆実行など関数群

### 前提となるパッケージのインストール手順

なし。
「CC環境構築手順書」で構築されたラズパイ環境で実行可能です。


### アプリケーション実行手順

#### MissionPlannerでの確認

CC環境構築手順書の3～5を行った上で、`day3_homework_original_app/original_mission.py`を実行します。

1. VSCodeでラズパイに接続
2. MissionPlannerでシミュレータ起動
3. ラズパイへテレメトリ転送
4. ラズパイ側のテレメトリルーティングを開始
5. `day3_homework_original_app/original_mission.py`を実行（VSCodeでファイル名タブ右の再生アイコンをクリック）
