from pymavlink import mavutil
import time
import datetime
import threading
from enum import Enum

class Rotation(Enum):
  CLOCKWISE = 1
  ANTICLOCKWIZE = -1

class Offset(Enum):
  ABSOLUTE = 0
  RELATIVE = 1

def print_with_time(message: str):
  print(f"{datetime.datetime.now()}: {message}")

# コンテキストマネージャ（master.closeを自動に実施）
class MasterConnection:
  def __init__(self, addr, src_sys, src_com):
    self.addr = addr
    self.src_sys = src_sys
    self.src_com = src_com

  def __enter__(self):
    self.master = mavutil.mavlink_connection(
      self.addr, self.src_sys, self.src_com
    )
    self.master.wait_heartbeat()
    print_with_time("接続しました")
    return self.master
  
  def __exit__(self, exc_type, exc_value, traceback):
    self.master.close()

def connect(conn, src_sys, src_com):
  master = mavutil.mavlink_connection(
    conn, src_sys, src_com
  )
  master.wait_heartbeat()
  print_with_time("接続しました")
  return master

def wait_until_be_(master, mode):
  master.set_mode_apm(master.mode_mapping()[mode])
  while True:
    if master.flightmode == mode:
      break
  print_with_time("GUIDEDモードへ移行")

def arm(master):
  master.arducopter_arm()
  master.motors_armed_wait()
  print_with_time("アームしました")

def arm_with_getting_latlon(master):
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
  arm(master)

  return results


def take_off(master, alt):
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
     0, 0, 0, 0, 0, 0, 0, alt
  )

def change_messaging_rate(master, key, rate, mask = [0, 0, 0, 0, 0]):
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, key, rate, *mask
  )

def check_alt(target_alt, current_alt):
  return current_alt >= target_alt * 0.95

def check_position(target_lat, target_lon, target_alt, current_lat, current_lon, current_alt):
  eps = 10e-6
  return (
    abs(current_lat - target_lat) < eps 
    and abs(current_lon - target_lon) < eps 
    and abs(current_alt - target_alt) < 0.5
  )

def wait_until_be_reachd(master, lat=None, lon=None, alt=None):
  while True:
    msg = master.recv_match(
      type="GLOBAL_POSITION_INT", blocking=True)
    if msg == None:
      continue

    current_lat = msg.lat / 1e7
    current_lon = msg.lon / 1e7
    current_alt = msg.relative_alt / 1000.0

    if (lat == None or lon == None):
      if check_alt(alt, current_alt):
        print_with_time("目標高度に到達しました")
        break
      print_with_time(f"高度: {current_alt}")
    else:
      if check_position(lat, lon, alt, current_lat, current_lon, current_alt):
        print_with_time("目標位置に到達しました")
        break
      print_with_time(f"位置: N{current_lat} E{current_lon} ALT: {current_alt}")

    time.sleep(0.1)

def go_to_(master, lat, lon, alt):
  master.mav.set_position_target_global_int_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b0000111111111000,
    int(lat * 1e7), int(lon * 1e7), alt,
    0, 0, 0, 0, 0, 0, 0, 0, 0,)
  
def command_yaw(master, angle: int, speed: int, rot: Rotation, offset: Offset):
  msg = master.mav.command_long_encode(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
    angle, speed, rot.value, offset.value, 0, 0, 0
  )
  master.mav.send(msg)

