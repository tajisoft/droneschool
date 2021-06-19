from dronekit import Vehicle, connect

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# vehicle.home_locationに値が設定されるまで
# downloadを繰り返し実行する
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    if not vehicle.home_location:
        print("ホームロケーションを待っています…")

# ホームロケーションの取得完了
print("ホームロケーション: %s" % vehicle.home_location)
