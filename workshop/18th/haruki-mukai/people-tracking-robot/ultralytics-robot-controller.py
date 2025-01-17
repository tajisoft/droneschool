from ultralytics import YOLO
from pymavlink import mavutil
# import time

machineId="/dev/ttyTHS1"
baudrate="56400"


def set_pwm(array):
    # チャンネルの値を設定
    sysid = 1  # 任意のシステムID
    compid = 1  # 任意のコンポーネントID
    sendValue = [1500] * 8 # 8つのチャンネルを全て1500で初期化

    for each in array:
        sendValue[each.get("num")] = each.get("value") * 4

    # RC_CHANNELS_OVERRIDEコマンドを送信
    print(sendValue)
    msg = master.mav.rc_channels_override_encode(sysid, compid, *sendValue)
    master.mav.send(msg)


def stop():
    set_pwm([{"num": 0, "value": 0}, {"num": 1, "value": 375}, {"num": 2, "value": 0}, {"num": 3, "value": 375}, {
            "num": 4, "value": 0}, {"num": 5, "value": 0}, {"num": 6, "value": 0}, {"num": 7, "value": 0}])
            




master = mavutil.mavlink_connection(machineId, baud=baudrate)
print("wait_heartbeat")
master.wait_heartbeat()
print("Hello!")
master.arducopter_disarm()






def init_mode_and_arm():
    print("init")
    # Choose a mode
    mavlink_mode = 'MANUAL'

    print(master.mode_mapping())
    # Check if mode is available
    if mavlink_mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mavlink_mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)

    # modeのidを取得
    mode_id = master.mode_mapping()[mavlink_mode]
    # modeのidを送信(manualモードに設定)
    master.set_mode(mode_id)

    master.arducopter_disarm()
    # master.motors_disarmed_wait()
    # message = master.mav.command_long_encode(
    #     master.target_system,  # Target system ID
    #     master.target_component,  # Target component ID
    #     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
    #     0,  # Confirmation
    #     # param1: Message ID to be streamed
    #     mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    #     1000000,  # param2: Interval in microseconds
    #     0,       # param3 (unused)
    #     0,       # param4 (unused)
    #     0,       # param5 (unused)
    #     0,       # param5 (unused)
    #     0        # param6 (unused)
    # )


init_mode_and_arm()

master.arducopter_arm()





# Load the model and run the tracker with a custom configuration file
model = YOLO("yolo11n.pt")
results = model.track(source=0,stream=True)
for result in results:
    isFind = False
    for index, v in enumerate(result.boxes.cls):
        if v == 0.0:
            isFind = True
            pos = result.boxes.xywh[index][0]-320
            print(pos)
            if result.boxes.xywh[index][3] > 400:
                print("super near")
                stop()
            else:
    	        set_pwm([{"num": 0, "value": 0}, {"num": 1, "value": 375+int(pos/5)}, {"num": 2, "value": 0}, {"num": 3, "value": 435}, {
            "num": 4, "value": 0}, {"num": 5, "value": 0}, {"num": 6, "value": 0}, {"num": 7, "value": 0}])
    if not isFind:
        stop()
    # result.save_txt("./runs/text")
    # times = time.time()
    # fileName = "/robot-controll/detected/detect-"+str(times)+".png"
    # result.save(filename=fileName)
