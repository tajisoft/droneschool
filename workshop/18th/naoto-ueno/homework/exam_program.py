from pymavlink import mavutil
import time
import math
import utils
import conn

# const
ADDR = "127.0.0.1:14550"
SRC_SYS_NO = 1
SRC_COM_NO = 9
MODE_GUIDED = "GUIDED"
TARGET_ALT = 4
GLOBAL_POSITION_INT = 33

# tree positions(xy)
trees = [
  [-7, 8],
  [1, 4],
  [10, 15],
]
RADIUS = 3

# flight
with conn.MasterConnection(ADDR, SRC_SYS_NO, SRC_COM_NO) as master:
  conn.wait_until_be_(master, MODE_GUIDED)
  origin_latlon = conn.arm_with_getting_latlon(master)
  tree_lat_lon = [utils.get_center_lat_lon(origin_latlon[0], origin_latlon[1], x, y) for x, y in trees]
  print(f"樹木の座標: {tree_lat_lon}")
  conn.take_off(master, alt = TARGET_ALT)
  conn.change_messaging_rate(master, GLOBAL_POSITION_INT, rate=10e5, mask = [0, 0, 0, 0, 0])
  conn.wait_until_be_reachd(master, alt = TARGET_ALT)

  # main loop（樹木ごと）
  for center_lat, center_lon in tree_lat_lon:
    wps = utils.create_circle(center_lat=center_lat, center_lon=center_lon, radius=RADIUS, altitute=TARGET_ALT, points=8)
    print(f"ウェイポイント: {wps}")
    # sub loop（樹木の周囲のウェイポイントごと）
    for wp_lat, wp_lon, alt in wps:
      conn.go_to_(master, wp_lat, wp_lon, TARGET_ALT)
      conn.wait_until_be_reachd(master, wp_lat, wp_lon, TARGET_ALT)