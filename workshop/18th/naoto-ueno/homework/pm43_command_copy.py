import conn
from conn import Rotation, Offset

# const
ADDR = "127.0.0.1:14550"
SRC_SYS_NO = 1
SRC_COM_NO = 9
MODE_GUIDED = "GUIDED"
TARGET_ALT = 4

# Message code
GLOBAL_POSITION_INT = 33

# flight
with conn.MasterConnection(ADDR, SRC_SYS_NO, SRC_COM_NO) as master:
  conn.wait_until_be_(master, MODE_GUIDED)
  conn.arm(master)
  conn.take_off(master, alt = TARGET_ALT)
  conn.change_messaging_rate(master, GLOBAL_POSITION_INT, rate=10e5, mask = [0, 0, 0, 0, 0])
  conn.wait_until_be_reachd(master, alt = TARGET_ALT)
  conn.command_yaw(master, 180, 10, Rotation.ANTICLOCKWIZE, Offset.RELATIVE)
