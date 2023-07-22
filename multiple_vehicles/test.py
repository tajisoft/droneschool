import math
import time

from dronekit import connect, LocationGlobalRelative

vehicle = connect('127.0.0.1:14581', wait_ready=False, timeout=60)
vehicle.wait_for_mode('GUIDED')