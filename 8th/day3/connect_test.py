# Day3に書いたコード
# シャープマークから始まる行はコメントです。
# プログラムとして実行対象外です。

# 使用する関数、クラスを宣言する
from dronekit import connect, VehicleMode
# 標準ライブラリのtimeを使う宣言
import time

# 機体に接続する
# シミュレータに接続する場合
vehicle = connect('127.0.0.1:14550', wait_ready=True)
# UbuntuでUSB経由で接続する場合
# vehicle = connect('/dev/ttyACM0,115200', wait_ready=True)

# 機体のパラメータを表示、ここではRTL_ALT（RTL高度）を出力
print('%d' % vehicle.parameters['RTL_ALT'])

# もしもRTL高度が20m未満だった場合、30mに設定する
rtlAlt = vehicle.parameters['RTL_ALT']
if rtlAlt < 2000:
    vehicle.parameters['RTL_ALT'] = 3000  # 単位cm

# 再度表示して変更されているか確認する
print('%d' % vehicle.parameters['RTL_ALT'])

# GUIDEDモードに変更する
vehicle.mode = VehicleMode('GUIDED')

# 機体をアームする
vehicle.armed = True

# 10秒処理をスリープ（停止）する
time.sleep(10)

# ここでプログラム終了
