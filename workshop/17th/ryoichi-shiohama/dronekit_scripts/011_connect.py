import time
from dronekit import connect

vehicle = connect('tcp:10.0.2.135:5762', wait_ready=True, timeout=60)

while True:
  print ("============================")
  print ("home_location: %s" % vehicle.home_location )
  print ("heading: %s" % vehicle.heading )
  print ("gimbal: %s" % vehicle.gimbal)
  print ("airspeed: %s" % vehicle.airspeed )
  print ("groundspeed: %s" % vehicle.groundspeed )
  print ("mode: %s" % vehicle.mode )
  print ("armed: %s" % vehicle.armed )
  time.sleep(1)
