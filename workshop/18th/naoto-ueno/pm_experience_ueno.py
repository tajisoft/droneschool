from pymavlink import mavutil
import time
import math
import utils
import threading

# const
HEIGHT = 10
conn = "127.0.0.1:14550"

# trees xy
trees = [
  [-3, 2],
  [1, 4],
  [5, 5],
]

# connection setup
master: mavutil.mavfile = mavutil.mavlink_connection(
  conn, source_system=1, source_component=90)
master.wait_heartbeat()

# change to "guided"
mode = 'GUIDED'
master.set_mode_apm(master.mode_mapping()[mode])

# confirm mode change
while True:
  if master.flightmode == mode:
    break
  master.recv_msg()

# get home point
def receive_home_position(master: mavutil.mavfile, results):
  while True:
    msg = master.recv_match(type='HOME_POSITION', blocking=True)
    if msg is not None:
      home_lat = msg.latitude / 1e7
      home_lon = msg.longitude / 1e7
      
      results[0] = home_lat
      results[1] = home_lon
      break

results = [0, 0]
thread = threading.Thread(target=receive_home_position, args=(master, results))
thread.start()

# arm
master.arducopter_arm()
master.motors_armed_wait()

thread.join()
print("armed")

# center list of trees
tree_lat_lon = [utils.get_center_lat_lon(results[0], results[1], x, y) for x, y in trees]
print(tree_lat_lon)

# takeoff
master.mav.command_long_send(
  master.target_system, master.target_component,
  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
  0, 0, 0, 0, 0, 0, 0, HEIGHT)

master.mav.command_long_send(
  master.target_system, master.target_component,
  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
  0, 33, 100000, 0, 0, 0, 0, 0)

# ascending
while True:
  received_msg = master.recv_match(
    type='GLOBAL_POSITION_INT', blocking=True)
  current_height = received_msg.relative_alt / 100

  if current_height >= HEIGHT * 0.95:
    print("Arrived at the target height")
    break

  time.sleep(0.1)

# main loop
print(tree_lat_lon)
for lat, lon in tree_lat_lon:
  wps = utils.create_circle(center_lat=lat, center_lon=lon, radius=3, altitute=10, points=8)
  print(wps)
  for lat, lon, alt in wps:
    master.mav.set_position_target_global_int_send(
      0, master.target_system, master.target_component,
      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # グローバル座標、相対高度
      0b0000111111111000,
      int(lat * 1e7), int(lon * 1e7), alt,
      0, 0, 0, 0, 0, 0, 0, 0, 0,)
    
    while True:
      msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
      if msg:
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000.0
        print(f"Current position: lat={current_lat}, lon={current_lon}, alt={current_alt}")

        # 目的地に近いかどうかを判定
        if abs(current_lat - lat) < 0.0001 and abs(current_lon - lon) < 0.000001 and abs(current_alt - alt) < 0.5:
            print("Approaching target position, stopping logging.")
            break
            # stop_event.set()
      time.sleep(0.1)

master.close()

