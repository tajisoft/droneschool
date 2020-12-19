# encoding: utf-8
import sys
import time
import requests

from dronekit import connect, VehicleMode, Command

mission_url = "https://tajisoft.jp/mission-10th.waypoints"


def download_mission(url):
    mis_req = requests.get(url)

    if mis_req.status_code != 200:
        print("Fail to download mission file.")
        return None

    print(mis_req.text)
    return mis_req.text

def mission_parser(mis_text):
    if mis_text is None or "QGC WPL 110" not in mis_text:
        return None
    
    wp_list = []
    for line in mis_text.split("\r\n"):
        if "QGC WPL 110" in line:
            continue
        wp_items = line.encode().split("\t")
        print(wp_items)
        if len(wp_items) <= 1 or wp_items[0] == "":
            continue
        ln_index=int(wp_items[0])
        ln_currentwp=int(wp_items[1])
        ln_frame=int(wp_items[2])
        ln_command=int(wp_items[3])
        ln_param1=float(wp_items[4])
        ln_param2=float(wp_items[5])
        ln_param3=float(wp_items[6])
        ln_param4=float(wp_items[7])
        ln_param5=float(wp_items[8])
        ln_param6=float(wp_items[9])
        ln_param7=float(wp_items[10])
        ln_autocontinue=int(wp_items[11].strip())
        cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
        wp_list.append(cmd)
    
    return wp_list

def connect_vehicle(dest_str):
    return connect(dest_str, wait_ready = True)

def upload_mission(vehicle, missions):
    # アップロードする前に既存のミッションをクリア
    cmds = vehicle.commands
    cmds.clear()
    # ミッションを追加してアップロード
    for command in missions:
        cmds.add(command)
    print("ミッションをアップロード")
    vehicle.commands.upload()

def start_fly(v):
    while not v.is_armable:
        print("機体の準備ができていません。")
        time.sleep(1)
    
    v.mode = VehicleMode("STABILIZE")
    v.armed = True
    while not v.armed:
        print("アーム状態ではありません。")
        time.sleep(1)

    v.commands.next = 0
    v.mode = VehicleMode("AUTO")
    v.channels.overrides["3"] = 1500
    time.sleep(1)
    print("ミッション航行を開始")

def wait_mission_complete(v):
    while True:
        if v.commands.next >= 225:
            print("ミッション完了")
            v.mode = VehicleMode("RTL")
            break
        else:
            time.sleep(1)


if __name__ == "__main__":
    # まだ文字列の状態
    missions = mission_parser(download_mission(mission_url))

    if missions is not None:
        vehicle = connect_vehicle("tcp:192.168.11.16:5773")
        upload_mission(vehicle, missions)
        start_fly(vehicle)
        wait_mission_complete(vehicle)
    else:
        print("ミッションファイルが不正です。")