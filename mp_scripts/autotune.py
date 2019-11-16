# -*- ecoding: utf-8 -*-
####################################################
# サンプルコード
# スティックアーミングをスクリプトで実装
####################################################
import clr
import time
import System
from System import Byte

clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp
clr.AddReference("MAVLink")
import MAVLink

print('Start Script')

# CH7をAUTOTUNEに割り当て
Script.ChangeParam('CH7_OPT', 17)

# 5秒スリープしてGPSロックを待つ
Script.Sleep(5000)
# GPS情報がセットされるまで1秒待つ
while cs.lat == 0:
    print 'Waiting for GPS'
    Script.Sleep(1000)
print('Got GPS')

# Guidedモードに設定
Script.ChangeMode('Guided')
# 30mにテイクオフ
MAV.doARM(True)
MAV.doCommand(MAVLink.MAV_CMD.TAKEOFF, 0, 0, 0, 0, 0, 0, 30);
while cs.alt < 28:
	Script.Sleep(100)

# PosHoldモードに変更したのち、AUTOTUNE開始
Script.ChangeMode('PosHold')
Script.SendRC(3,1500,True)
Script.Sleep(1000)
Script.SendRC(7,2000,True)

print('AUTOTUNE 開始しました。')