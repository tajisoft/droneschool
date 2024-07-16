from pymavlink import mavutil

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
  "tcp:10.0.2.135:5762",
  source_system=1,
  source_component=90
)
print("接続を試みています...")
master.wait_heartbeat()
print("ハートビートを受信しました。接続に成功しました。")

# メッセージレート変更
# msg = master.mav.request_data_stream_encode(
#   master.target_system,
#   master.target_component,
#   0, 10, 1
# )
# master.mav.send(msg)

try:
  while True:
    print("メッセージを待っています...")
    # HOME_POSITIONメッセージを受信、アームのタイミングで受信できる
    msg = master.recv_match(type='HOME_POSITION', blocking=False)
    print("メッセージの取得を試みました。")
    if msg is not None:
      # ホームロケーション情報を設定
      home_latitude = msg.latitude / 1.0e7
      home_longitude = msg.longitude / 1.0e7
      home_altitude = msg.altitude / 1000.0 # メートル単位に変換
      
      print(
        f"ホームロケーション: lat={home_latitude}, long={home_longitude}, alt={home_altitude}"
      )
      break # ホームロケーションを取得したら終了
    else:
      print("メッセージがありません。")
except Exception as e:
  print(f"エラーが発生しました: {e}")

