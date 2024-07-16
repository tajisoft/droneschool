import time
from pymavlink import mavutil

def main():
    master = connect_vehicle("127.0.0.1:14551")
    mode_change(master,'GUIDED')
    height = int(input("離陸します。離陸高度を入力してください。"))
    arming_vehicle(master)
    takeoff(master,height)

    


# 機体接続
def connect_vehicle(device_ip):
    master: mavutil.mavfile = mavutil.mavlink_connection(
        device = device_ip, source_system=1, source_component=90
    )

    #HEARTBEATをまつ
    master.wait_heartbeat()
    print("機体への接続完了")
    return master

# ARM
def arming_vehicle(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
)                

def mode_change(master,operation_mode):
    # GUIDEDモードに変更
    
    master.set_mode_apm(master.mode_mapping()[operation_mode])

    # モード変更を確認
    while True:
        if master.flightmode == operation_mode:
            break
        master.recv_msg()

def takeoff(master,takeoff_height):
    # 離陸
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, takeoff_height
    )

    # メッセージレート変更: GLOBAL_POSITION_INT(33)を10Hzで受信
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, 33, 100000, 0, 0, 0, 0, 0
    )

    
# 目標高度への到達を確認
    while True:
        # GLOBAL_POSITION_INTから相対高度を取得
        received_msg = master.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True
        )
        current_altitude = received_msg.relative_alt / 1000

        print("高度: {}".format(current_altitude))

        if current_altitude >= takeoff_height * 0.95:
            print("目標高度に到達")
            break
    
        time.sleep(0.1)

    
    





if __name__ == '__main__':
    main()
