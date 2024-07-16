# Hello, ArduPilot!  

# コース２　宿題  
- 207_simplegot.pyを写経  
アーム、モード変更、離陸から行えるようにアレンジしました  
- pm00_connect_heartbeat.pyを写経  
- 209_position.pyを写経  
- 213_mission_modify.pyを写経  
ミッションの読み込みチェックを行うようにアレンジしました。  
- move_by_input_meter.pyを追加  
コマンドラインでx、y軸方向の距離を入力、その地点までDronekitのsimple_goto()を使用して移動するスクリプトを作成  
シミュレータで動作確認済み  

# コース2終了課題  
## 概要  
Dronekit-pythonを使用。離陸後、コマンドラインで入力された機体の正面に対する距離(x, y)の位置に移動する指令を送信する  
(x, y) [m]:　機体の正面がy（正面が正), 直行する方向がx(右向き正)  
## 必要ライブラリ
Dronekit-python  
## アプリケーション実行手順  
- MissonPlannerでシミュレーションを起動  
- コマンドラインでスクリプトを実行、目標の距離を入力  
```
python move_by_input_meter.py

#example
#Please input moving point in x,y format (meter) (type 'quit' to finish):5,5
```
- 入力された位置に移動した後、停止
