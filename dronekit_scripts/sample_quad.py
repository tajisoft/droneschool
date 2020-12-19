# encoding: utf-8
# パッケージインポート
from dronekit import connect, VehicleMode
import time

# WindowsPCに接続されている実機に接続
# もし、'module' object has no attribute 'Serial'のようなエラーが出る場合は、
# pip uninstall serial
# pip uninstall pyserial
# pip install pyserial
# を試す。
vehicle = connect("COM3", wait_ready=True)

# GUIDEDモードにしてアーム
vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = True

time.sleep(5)

vehicle.armed = False

vehicle.close()