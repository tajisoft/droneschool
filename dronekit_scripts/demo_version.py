from dronekit import connect
import time

vehicle = connect("/dev/ttyS3", wait_ready=True);

print("Version %s" % vehicle.version)

# callback test
def homeloc_callback(self, name, val):
    print("Location(Global):%s -> %s" % (name, val))

vehicle.add_attribute_listener('location.global_frame', homeloc_callback)

time.sleep(10)

vehicle.remove_attribute_listener('location.global_frame', homeloc_callback)
