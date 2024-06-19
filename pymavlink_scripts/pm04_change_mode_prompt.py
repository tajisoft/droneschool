import threading
import time
from pymavlink import mavutil
import sys

def set_mode_interactively(master:mavutil.mavfile):
    while True:
        mode = input("Enter mode name: ").strip().upper()
        
        # モードが有効かをチェック
        if mode not in master.mode_mapping():
            print('Unknown mode: {}'.format(mode))
            print('Try:', list(master.mode_mapping().keys()))
            continue
        
        # モードを設定
        master.set_mode(master.mode_mapping()[mode])

        # モード変更の確認を行う
        while True:
            master.recv_msg()
            if master.flightmode == mode:
                break

        print("変更後モード:", master.flightmode)

def main():
    # 機体への接続
    master:mavutil.mavfile = mavutil.mavlink_connection(
        "127.0.0.1:14551", source_system=1, source_component=90)
    master.wait_heartbeat()

    # 対話式にモードを設定
    set_mode_interactively(master)

if __name__ == "__main__":
    main()
