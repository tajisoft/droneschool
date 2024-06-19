from pymavlink import mavutil
import math
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    device="127.0.0.1:14551", source_system=1, source_component=90)
master.wait_heartbeat()


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


# MAVLinkメッセージを生成する
msg = master.mav.set_attitude_target_encode(
    0,      # ブートからの時間（今回は未使用）
    master.target_system, master.target_component,    # ターゲットシステム、コンポーネント
    0b00000000 if False else 0b00000100,  # マスク
    to_quaternion(20, -20, 0),  # クオータニオン角度(ロール,ピッチ,ヨー角度)
    0, 0, math.radians(0),  # ロール,ピッチ,ヨーレート
    0.5  # スラスト
)

# MAVLinkメッセージ送信
for x in range(0, 10):
    master.mav.send(msg)
    time.sleep(0.1)
