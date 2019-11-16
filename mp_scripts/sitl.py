# -*- ecoding: utf-8 -*-
####################################################
# サンプルコード
# SITL接続
####################################################
import os
import sys
import math
import clr
import time
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities") # includes the Utilities class
clr.AddReference("MissionPlanner.ExtLibs.Comms")
clr.AddReference("System")
import MissionPlanner.ExtLibs.Comms
import System

from System.Diagnostics import Process

for i in range(20):
    workdir = r'C:\Users\tajis\Documents\Mission Planner\sitl\d' + str(i)
    print(workdir)
    if not os.path.exists(workdir):
        os.makedirs(workdir)
    proc = Process()
    proc.StartInfo.WorkingDirectory = workdir
    proc.StartInfo.FileName = r'C:\Users\tajis\Documents\Mission Planner\sitl\ArduCopter.exe'
    proc.StartInfo.Arguments = ' -M+ -s1 --uartA tcp:0 --defaults ..\default_params\copter.parm --instance ' + str(i) + ' --home -35.363261,'+ str(149.165330 + 0.000001 * i) +',584,353'
    proc.Start()

    port = MissionPlanner.ExtLibs.Comms.TcpSerial()
    port.client = System.Net.Sockets.TcpClient("127.0.0.1", 5760 + 10 * i)

    mav = MissionPlanner.MAVLinkInterface()
    mav.BaseStream = port
    mav.getHeartBeat()
    MissionPlanner.MainV2.Comports.Add(mav)