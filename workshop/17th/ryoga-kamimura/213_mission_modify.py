from dronekit import Command, connect, VehicleMode
from pymavlink import mavutil
import time

CONNECTION_STRING = 'tcp:127.0.0.1:5762'
vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)

#コマンドオブジェクトの取得
cmds = vehicle.commands

#ダウンロード実行
cmds.download()
cmds.wait_ready()

#編集用に別のリストにミッションを保持
missionList = []
for cmd in cmds:
    missionList.append(cmd)

#ミッションの編集
#ここでは、最初のミッションをテイクオフに変更
#ミッションが保持されているかチェックする
if missionList:
    missionList[0].command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
else:
    print('Failed to load mission')

#現在のミッションをクリアする
cmds.clear()

#変更したミッションをアップロード
for cmd in missionList:
    cmds.add(cmd)
    cmds.upload()