# 実機接続に変更
# ※実機接続でT10JからSTABILIZE->ALT_HOLD->LOITER->RTL->LOITER変更で確認
# 元々記載されているRTL_ALTって何？->何を設定すればいいのかわからない
# 以下の説明見てもよくわからない
# https://dronekit.netlify.app/guide/vehicle_state_and_parameters.html
import time
from dronekit import Vehicle, connect

# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
vehicle = connect('127.0.0.1:14552', wait_ready=True, timeout=60)

#RTL_ALTが変更された際に通知を受け取る
#デコレーターの設定
#@vehicle.parameters.on_attribute('RTL_ALT') # 確認できず
#@vehicle.parameters.on_attribute('RTL_LOITER') # 確認できず
@vehicle.parameters.on_attribute('LOITER') # 確認できず
def decorated_RTL_ALT_callback(self, attr_name, value):
	print("デコレーター: %s が %s に変更されました！" % (attr_name, value))

while True:
    time.sleep(1)
