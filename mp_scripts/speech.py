# -*- ecoding: utf-8 -*-
####################################################
# サンプルコード
# speed機能
####################################################
import clr
import time
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities") # includes the Utilities class

print('Start Script')

MissionPlanner.MainV2.speechEnable = True

while True:
	print('speech...')
	MissionPlanner.MainV2.speechEngine.SpeakAsync("ロール " + cs.roll.ToString())
    time.sleep(1)

print('End Script')