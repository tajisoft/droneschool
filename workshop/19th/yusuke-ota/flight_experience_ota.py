import argparse
from pymavlink import mavutil
import time
from math import radians, cos, sin, pi, sqrt, atan2
from geopy.distance import distance
from geopy.point import Point

ARMTEST = False  # テストモードを有効にする
WPSPEED = 3.0  # WPNAV_SPEEDのデフォルト値（m/s）
FC_SYSID = 1  # 初期値（Mission Planner以外のシステムID）
FC_COMPID = 1  # 初期値（Mission Planner以外のコンポーネントID）

def wait_fc_heartbeat(master):
    global FC_SYSID, FC_COMPID
    while True:
        msg = master.recv_match(type="HEARTBEAT", blocking=True)
        if msg.get_srcSystem() != 255:  # Mission Planner以外
            FC_SYSID = msg.get_srcSystem()
            FC_COMPID = msg.get_srcComponent()
            break
    return msg

def recv_match_filtered(master, filter_fn, *args, **kwargs):
    """
    指定されたフィルター関数に合致するメッセージを受信する
    Args:
        master (mavutil.mavfile): 接続済みのMAVLinkオブジェクト
        filter_fn (callable)
            フィルター関数。メッセージを引数に取り、True/Falseを返す。
        *args, **kwargs: mavutil.recv_matchの引数
    Returns:
        mavutil.mavlink.MAVLink_message: フィルターに合致したメッセージ
    """
    msg = master.recv_match(*args, **kwargs)
    if msg and filter_fn(msg):
        return msg
    return None

def is_from_fc(msg):
    """
    メッセージがFCからのものであるかを確認する
    Args:
        msg (mavutil.mavlink.MAVLink_message): チェックするメッセージ
    Returns:
        bool: FCからのメッセージであればTrue、そうでなければFalse
    """
    return msg.get_srcSystem() == FC_SYSID and msg.get_srcComponent() == FC_COMPID

def recv_from_fc(master, *args, **kwargs):
    """
    FCからのメッセージを受信するためのラッパー関数
    Args:
        master (mavutil.mavfile): 接続済みのMAVLinkオブジェクト
        *args, **kwargs: mavutil.recv_matchの引数
    Returns:
        mavutil.mavlink.MAVLink_message: FCからのメッセージ
    """
    return recv_match_filtered(master, is_from_fc, *args, **kwargs)

def resolve_enum_short(enum_type: str, value: int):
    """
    Resolves the short name of an enum value from a MAVLink enum type.

    Args:
        enum_type (str): The name of the MAVLink enum type (e.g., "MAV_MODE").
        value (int): The integer value of the enum.

    Returns:
        str: The short name of the enum value with the enum type prefix removed,
             or a string indicating the value is unknown if not found.

    Example:
        >>> resolve_enum_short("MAV_MODE", 1)
        'MANUAL_ARMED'
        >>> resolve_enum_short("MAV_MODE", 999)
        'UNKNOWN MAV_MODE: 999'
    """
    try:
        full_name = mavutil.mavlink.enums[enum_type][value].name
        prefix = enum_type.upper() + "_"
        return full_name.removeprefix(prefix)  # Python 3.9+
    except KeyError:
        return f"UNKNOWN {enum_type}: {value}"

def connect_to_mavlink(connection_string):
    """
    Connect to a MAVLink device using the specified connection string.

    Args:
        connection_string (str): The connection string for the MAVLink device.

    Returns:
        mavutil.mavfile: The connected MAVLink object.
    """
    master = mavutil.mavlink_connection(connection_string, source_system=1, source_component=90)
    print(f"Waiting for heartbeat on {connection_string}...")
    hb = wait_fc_heartbeat(master)
    print(f"Heartbeat received from FC: sysid={FC_SYSID}, compid={FC_COMPID}")
    # デバッグ用の情報を表示(MAVProxyやMAVLink Router経由で接続した場合、MAVTypeがおかしくなることがあるので注意)
    print(f"target_system: {master.target_system}, target_component: {master.target_component}")
    print("MAV type:", resolve_enum_short("MAV_TYPE", hb.type))
    # print("Mode mapping:")
    # for name, mode_id in master.mode_mapping().items():
    #     print(f"  {name}: {mode_id}")
    return master

def resolve_msg_id(name: str) -> int:
    const_name = f"MAVLINK_MSG_ID_{name.upper()}"
    if hasattr(mavutil.mavlink, const_name):
        return getattr(mavutil.mavlink, const_name)
    else:
        raise ValueError(f"Unknown message name: {name}")

def request_message(master, msg_type: str, interval_us: int = 100000):
    """
    指定したメッセージタイプを指定間隔で受信するようにリクエストする
    
    Args:
        master (mavutil.mavfile): 接続済みのMAVLinkオブジェクト
        msg_type (str): リクエストするメッセージのタイプ（例: "GLOBAL_POSITION_INT"）
        interval_us (int): 受信間隔（マイクロ秒）
    """
    msg_id = resolve_msg_id(msg_type)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # confirmation
        msg_id,  # message ID
        interval_us,  # interval in microseconds
        0, 0, 0, 0, 0  # unused parameters
    )
    
def request_message_and_wait(master, msg_type: str, interval_us: int = 100000, timeout=5.0):
    """
    指定したメッセージタイプをリクエストし、指定時間内に受信できるか確認する
    Args:
        master (mavutil.mavfile): 接続済みのMAVLinkオブジェクト
        msg_type (str): リクエストするメッセージのタイプ（例: "GLOBAL_POSITION_INT"）
        interval_us (int): 受信間隔（マイクロ秒）
        timeout (float): タイムアウト時間（秒）
    Returns:
        bool: メッセージを受信できたかどうか
    """
    request_message(master, msg_type, interval_us)
    msg = master.recv_match(type=msg_type, blocking=True, timeout=timeout)  # 確認のために最初のメッセージを受信
    if msg is None:
        print(f"Warning: No initial message received for {msg_type}. Check connection or message type.")
        return False
    print(f"Requested {msg_type} at {1000000/interval_us} Hz.")
    return True


def request_param(master, param_id: str):
    """param_request_read_send を安全に送る"""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_id.encode("ascii").ljust(16, b'\x00'),
        -1
    )

def wait_param_value(master, param_id: str, timeout=3.0):
    """PARAM_VALUE を待つ"""
    start = time.time()
    while time.time() - start < timeout:
        msg = recv_from_fc(master, type="PARAM_VALUE", blocking=True, timeout=0.2)
        if msg and msg.param_id.strip('\x00') == param_id:
            return msg.param_value
    raise TimeoutError(f"Timeout while waiting for {param_id}")

def get_param(master, param_id: str, timeout=3.0):
    """パラメータの取得"""
    request_param(master, param_id)
    return wait_param_value(master, param_id, timeout)


def get_wpnav_speed(master, fallback=3.0):
    """
    WPNAV_SPEED（cm/s）を取得して m/s に変換

    Returns:
        float: 速度（m/s）
    """
    val = get_param(master, "WPNAV_SPEED")
    if val is not None:
        return val / 100.0
    print("Warning: WPNAV_SPEED not found. Using fallback value.")
    return fallback

def set_param(master, param_id: str, value: float, param_type: int, timeout=3.0):
    """パラメータの設定と反映確認"""
    param_bytes = param_id.encode("ascii").ljust(16, b'\x00')
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_bytes,
        value,
        param_type
    )
    return wait_param_value(master, param_id, timeout)


def set_and_verify_param(master, param_id: str, value: float, param_type: int, timeout=3.0):
    """パラメータの設定＋検証"""
    actual = set_param(master, param_id, value, param_type, timeout)
    if abs(actual - value) > 1e-3:
        raise ValueError(f"Mismatch: set {value} but got {actual}")
    return actual

def is_current_mode(master, mode_name):
    master.recv_msg()  # 最新のflightmodeに更新
    current = master.flightmode
    return current and current.strip().upper() == mode_name.strip().upper()

def change_mode_and_confirm(master, mode_name, timeout=10.0):
    """指定モードへ変更して、flightmodeで確認を行う"""
    print(f"Requesting mode change to: {mode_name}")
    master.set_mode(mode_name)

    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = recv_from_fc(master, type="HEARTBEAT", blocking=True, timeout=0.2)
        if msg and is_current_mode(master, mode_name):
            print(f"Mode confirmed: {mode_name}")
            return True
        time.sleep(0.1)  # 過負荷防止

    print(f"Timeout: Mode change to {mode_name} not confirmed.")
    return False

def arm_drone(master, timeout=10.0):
    print("Switching to GUIDED mode...")
    if not change_mode_and_confirm(master, "GUIDED", timeout=timeout):
        print("Failed to enter GUIDED mode.")
        return False

    print("Sending ARM command...")
    master.arducopter_arm()

    # ARM 状態確認（HEARTBEAT から armedフラグを見る）
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = recv_from_fc(master, type="HEARTBEAT", blocking=True, timeout=0.2)
        if msg and master.motors_armed():
            print("Drone is armed!")
            return True

    print("Timeout: Drone did not arm.")
    return False

def test_arm_disarm(master, wait_sec=2):
    print("=== ARM/DISARM TEST START ===")

    if not arm_drone(master):
        print("ARM failed.")
        return False

    print(f"Waiting for {wait_sec} seconds before disarming...")
    time.sleep(wait_sec)

    print("Sending DISARM command...")
    master.arducopter_disarm()

    start_time = time.time()
    while time.time() - start_time < 5.0:
        msg = recv_from_fc(master, type="HEARTBEAT", blocking=True, timeout=0.2)
        if msg and not master.motors_armed():
            print("Drone is disarmed!")
            print("=== ARM/DISARM TEST PASSED ===")
            return True

    print("DISARM failed or timed out.")
    print("=== ARM/DISARM TEST FAILED ===")
    return False

def takeoff_to_altitude(master, target_alt=5.0, timeout_per_meter=3.0, min_timeout=10.0):
    """
    指定高度までテイクオフし、到達を確認する（GUIDED+ARMED確認付き）

    Args:
        master (mavutil.mavfile): 接続済みのMAVLinkオブジェクト
        target_alt (float): 目標高度（メートル）
        timeout_per_meter (float): 1mあたりの待機秒数
        min_timeout (float): 最低タイムアウト秒数

    Returns:
        bool: 成功したかどうか
    """
    # --- モードがGUIDEDであるか確認 ---
    master.recv_msg()  # 最新化
    if not is_current_mode(master, "GUIDED"):
        print("Error: Current mode is not GUIDED.")
        return False

    # --- ARM状態を確認 ---
    if not master.motors_armed():
        print("Error: Drone is not armed.")
        return False

    timeout = max(target_alt * timeout_per_meter, min_timeout)
    print(f"Taking off to {target_alt:.1f}m with timeout {timeout:.1f}s...")

    # --- TAKEOFFコマンド送信 ---
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        target_alt
    )

    # --- 高度監視 ---
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = recv_from_fc(master, type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # mm → m
            print(f"  Current Altitude: {current_alt:.2f} m", end='\r')
            if current_alt >= target_alt * 0.95:
                print(f"\nTakeoff complete. Reached {current_alt:.2f} m.")
                return True

    print("\nTimeout: Takeoff did not reach target altitude.")
    return False

def send_guided_waypoint(master, lat, lon, alt):
    """GUIDEDモードで特定の地点へ移動させる"""
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # 位置制御のみ
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,  # velocity
        0, 0, 0,  # acceleration
        0, 0      # yaw, yaw_rate
    )

def compute_target_position(lat, lon, alt, offset_m, bearing_deg):
    """現在地とオフセットから目標位置を計算"""
    start_point = Point(lat, lon)
    dest_point = distance(meters=offset_m).destination(start_point, bearing_deg)
    return dest_point.latitude, dest_point.longitude, alt

def haversine(lat1, lon1, lat2, lon2):
    """2点間の距離（m）を求める"""
    R = 6371000  # 地球半径[m]
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    return R * 2 * atan2(sqrt(a), sqrt(1 - a))

def wait_until_position_reached(master, target_lat, target_lon, threshold_m=2.0, timeout=20.0):
    """指定位置に到達するまで待機"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = recv_from_fc(master, type="GLOBAL_POSITION_INT", blocking=True, timeout=0.2)
        if msg:
            cur_lat = msg.lat / 1e7
            cur_lon = msg.lon / 1e7
            dist = haversine(cur_lat, cur_lon, target_lat, target_lon)
            print(f"  → distance to target: {dist:.2f}m", end="\r")
            if dist <= threshold_m:
                print(f"\nArrived at target (within {threshold_m}m)")
                return True
        time.sleep(0.1)
    print("\nTimeout: Target not reached.")
    return False

def send_guided_single_waypoint(master, offset_m=10.0, bearing_deg=0.0, threshold_m=1.0):
    print(f"\n--- GUIDED waypoint test: {offset_m}m @ {bearing_deg}° ---")

    # 現在位置取得
    msg = recv_from_fc(master, type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if msg is None:
        print("Failed to get current position.")
        return False

    current_lat = msg.lat / 1e7
    current_lon = msg.lon / 1e7
    current_alt = msg.relative_alt / 1000.0

    # 目的地を計算
    target_lat, target_lon, target_alt = compute_target_position(
        current_lat, current_lon, current_alt, offset_m, bearing_deg
    )

    print(f"Current position: lat={current_lat:.7f}, lon={current_lon:.7f}, alt={current_alt:.2f}m")
    print(f"Target position:  lat={target_lat:.7f}, lon={target_lon:.7f}, alt={target_alt:.2f}m")

    # ウェイポイント送信
    send_guided_waypoint(master, target_lat, target_lon, target_alt)

    # 到達確認
    success = wait_until_position_reached(
        master, target_lat, target_lon,
        threshold_m=threshold_m,
        timeout=max(10.0, offset_m / 0.5)
    )

    print("Waypoint test succeeded." if success else "Waypoint test failed.")
    return success

def send_guided_waypoints_from_dict(master, waypoints_dict, threshold_m=1.0):
    """
    辞書形式の複数ウェイポイントにGUIDEDで順次移動

    Args:
        master: pymavlink 接続オブジェクト
        waypoints_dict (dict): {"1": {"lat":..., "lon":..., "alt":...}, ...}
        threshold_m (float): 各ポイント到達判定の許容誤差[m]

    Returns:
        bool: 全ウェイポイントに正常到達できたか
    """
    for key in sorted(waypoints_dict.keys(), key=lambda x: int(x)):
        wp = waypoints_dict[key]
        lat, lon, alt = wp["lat"], wp["lon"], wp["alt"]
        print(f"\n→ Moving to WP {key}: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m")
        send_guided_waypoint(master, lat, lon, alt)
        success = wait_until_position_reached(master, lat, lon, threshold_m=threshold_m)
        if not success:
            print(f"Failed to reach waypoint {key}.")
            return False
    print("\nAll waypoints reached.")
    return True

def compute_target_position_dict(base_lat, base_lon, base_alt, offset_m, bearing_deg):
    """
    現在地＋方位と距離から目標地点を計算し、辞書形式で返す

    Returns:
        dict: {"lat":..., "lon":..., "alt":...}
    """
    start_point = Point(base_lat, base_lon)
    dest = distance(meters=offset_m).destination(start_point, bearing_deg)
    return {
        "lat": dest.latitude,
        "lon": dest.longitude,
        "alt": base_alt
    }

def append_waypoint_dict(wp_dict, lat, lon, alt):
    """
    Waypoint辞書に次のIDで追加する
    """
    next_id = str(len(wp_dict) + 1)
    wp_dict[next_id] = {"lat": lat, "lon": lon, "alt": alt}


def generate_figure8_waypoints(center_lat, center_lon, alt, radius=10.0, points_per_circle=16, heading_deg=0.0):
    """
    8の字飛行のウェイポイントを生成（heading_degで全体回転）

    Args:
        center_lat (float): 中心緯度
        center_lon (float): 中心経度
        alt (float): 高度[m]
        radius (float): 各円の半径[m]
        points_per_circle (int): 各円の分割数（滑らかさ）
        heading_deg (float): 8の字全体の回転角度（北=0°, 時計回り）

    Returns:
        dict: ウェイポイント辞書 {"1": {"lat":..., "lon":..., "alt":...}, ...}
    """
    waypoints = {}
    center = Point(center_lat, center_lon)
    heading_rad = radians(heading_deg)

    # 左右円の中心をheading_deg方向に回転
    left_dx = -radius * cos(heading_rad)
    left_dy = -radius * sin(heading_rad)
    right_dx = radius * cos(heading_rad)
    right_dy = radius * sin(heading_rad)

    def offset_point(origin, dx, dy):
        p = distance(meters=abs(dx)).destination(origin, 90 if dx >= 0 else 270)
        p = distance(meters=abs(dy)).destination(p, 0 if dy >= 0 else 180)
        return p

    left_center = offset_point(center, left_dx, left_dy)
    right_center = offset_point(center, right_dx, right_dy)

    # 左円（0°→360°、反時計回り）
    for i in range(points_per_circle):
        theta = 2 * pi * i / (points_per_circle - 1)  # ← ここを修正
        dx = radius * cos(theta + heading_rad)
        dy = radius * sin(theta + heading_rad)
        p = offset_point(left_center, dx, dy)
        waypoints[str(i + 1)] = {"lat": p.latitude, "lon": p.longitude, "alt": alt}

    # 右円（180°→-180°、時計回り）
    for i in range(points_per_circle):
        theta = pi - 2 * pi * i / (points_per_circle - 1)  # ← 同様に修正
        dx = radius * cos(theta + heading_rad)
        dy = radius * sin(theta + heading_rad)
        p = offset_point(right_center, dx, dy)
        waypoints[str(i + 1 + points_per_circle)] = {"lat": p.latitude, "lon": p.longitude, "alt": alt}

    return waypoints

def test_guided_dict_waypoints(master, radius=10.0, threshold_m=1.0, points_per_circle=16, heading_deg=0.0):
    """
    現在地を中心に8の字ウェイポイントを生成し、順次移動するテスト

    Args:
        master: pymavlinkの接続オブジェクト
        radius (float): 各円の半径[m]
        threshold_m (float): 到達判定距離[m]

    Returns:
        bool: 全ウェイポイントへの到達成功可否
    """
    print(f"\n--- GUIDED 8の字ウェイポイントテスト: 半径 {radius}m ---")

    # 現在位置取得
    msg = recv_from_fc(master, type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if msg is None:
        print("Failed to get current position.")
        return False

    current_lat = msg.lat / 1e7
    current_lon = msg.lon / 1e7
    current_alt = msg.relative_alt / 1000.0

    print(f"Current position: lat={current_lat:.7f}, lon={current_lon:.7f}, alt={current_alt:.2f}m")

    # ウェイポイント生成
    wps = generate_figure8_waypoints(current_lat, current_lon, current_alt, radius=radius, points_per_circle=points_per_circle, heading_deg=heading_deg)


    # 順次移動
    success = send_guided_waypoints_from_dict(master, wps, threshold_m=threshold_m)

    print("Figure-8 test succeeded." if success else "Figure-8 test failed.")
    return success


def main():
    # 接続先のIPアドレスやポートをコマンドライン引数から取得
    parser = argparse.ArgumentParser(description="pymavlink connection example")
    parser.add_argument(
        "--connect",
        type=str,
        # default='tcp:172.30.32.1:5763',
        # default='tcp:172.26.176.1:5763',
        default='127.0.0.1:14551',
        help="Connection string (e.g. '127.0.0.1:14551' or '/dev/ttyTHS1')"
    )
    args = parser.parse_args()

    # MAVLinkに接続
    master = connect_to_mavlink(args.connect)
    
    # 受信メッセージの設定
    if not request_message_and_wait(master, "GLOBAL_POSITION_INT", interval_us=100000):
        print("Failed to request GLOBAL_POSITION_INT messages.")
        return

    # パラメータの設定と確認
    param_name = "WPNAV_SPEED"
    param_value = WPSPEED * 100  # m/s → cm/s
    param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    print(f"Setting {param_name} to {WPSPEED:.2f} m/s ({param_value:.0f} cm/s)...")
    actual_value = set_and_verify_param(master, param_name, param_value, param_type)
    print(f"Confirmed {param_name}: {actual_value / 100:.2f} m/s")

    # モード変更とARM/DISARMのテスト
    if ARMTEST:
        if not test_arm_disarm(master):
            print("ARM/DISARM test failed.")
        else:
            print("ARM/DISARM test succeeded.")
    else:
        print("ARMTEST is disabled, skipping ARM/DISARM test.")
        
        # 実際のテイクオフを行う流れ
    print("Preparing for takeoff...")
    time.sleep(3)  # 少し待つ
    if not change_mode_and_confirm(master, "GUIDED"):
        print("Failed to change to GUIDED mode.")
        return

    if not arm_drone(master):
        print("Failed to arm drone.")
        return

    if not takeoff_to_altitude(master, target_alt=5.0):
        print("Takeoff failed.")
    else:
        print("Takeoff succeeded.")

    print("Starting GUIDED movement test...")
    if not send_guided_single_waypoint(master, offset_m=5.0, bearing_deg=120.0):
        print("GUIDED test failed.")
    else:
        print("GUIDED test succeeded.")

    print("Starting 8の字飛行 test...")
    if not test_guided_dict_waypoints(master, radius=2.0, points_per_circle=16, heading_deg=20.0):
        print("8の字飛行に失敗しました")
    else:
        print("8の字飛行成功！")
    
    time.sleep(2)  # 少し待つ

    if not change_mode_and_confirm(master, "RTL"):
        print("Failed to change to RTL mode.")
        return

if __name__ == "__main__":
    main()

