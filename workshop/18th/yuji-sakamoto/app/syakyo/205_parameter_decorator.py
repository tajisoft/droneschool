# 実機接続に変更
# 元々記載されているRTL_ALTって何？->パラメータ設定名
# ・・・なのでパラメータの値を変更した時の通知処理
# https://dronekit.netlify.app/guide/vehicle_state_and_parameters.html
import time
from dronekit import Vehicle, connect

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
vehicle = connect('127.0.0.1:14552', wait_ready=True, timeout=60)

# RTL_ALTが変更された際に通知を受け取る
# MPなど別の接続からパラメータを変更して確認する
#デコレーターの設定
@vehicle.parameters.on_attribute('RTL_ALT') 
def decorated_RTL_ALT_callback(self, attr_name, value):
	print("デコレーター: %s が %s に変更されました！" % (attr_name, value))

while True:
    time.sleep(1)
