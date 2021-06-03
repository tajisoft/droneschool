from dronekit import connect
import time

# 接続
vehicle = connect('127.0.0.1:14550', wait_ready=True, timeout=60)

print("connected")

# parameter
print("RTL ALT is {}".format(vehicle.parameters["RTL_ALT"]))
vehicle.parameters["RTL_ALT"] = 2000
time.sleep(1)
print("new RTL ALT is {}".format(vehicle.parameters["RTL_ALT"]))


# vehicle.armed = True
# vehicle.mode = VehicleMode("GUIDED")



# # listener
# def location_callback(self, attr, val):
#     print(attr)
#     print(val)

# # Regist listener
# vehicle.add_attribute_listener("location.global_frame", location_callback)
# time.sleep(10)

# # Unregist listener
# vehicle.remove_attribute_listener("location.global_frame", location_callback)

vehicle.close()