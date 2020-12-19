# -*- ecoding: utf-8 -*-
####################################################
# サンプルコード
# スティックアーミング～離陸～着陸をスクリプトで実装
####################################################

print('Start Script')

# 1～9チャンネルをニュートラルに
for chan in range(1,9):
    Script.SendRC(chan,1500,False)
# スロットルをローに設定
Script.SendRC(3,Script.GetParam('RC3_MIN'),True)

# 5秒スリープしてGPSロックを待つ
Script.Sleep(5000)
# GPS情報がセットされるまで1秒待つ
while cs.lat == 0:
    print 'Waiting for GPS'
    Script.Sleep(1000)
print('Got GPS')

# モードスイッチをローに設定、この場合Stabilize
Script.SendRC(5,1000,True)
# スティックアームを送信
Script.SendRC(3,1000,False) # 第3引数のFalseの場合はすぐに送信しない
Script.SendRC(4,2000,True)
# メッセージをクリア
cs.messages.Clear()
# ”Arming motors”メッセージを待つ
Script.WaitFor('Arming motors',30000)
# ヨー方向のチャンネルをニュートラルに設定
Script.SendRC(4,1500,True)
print('Motors Armed!')

# スロットルを上げる
Script.SendRC(3,1700,True)
# 50m未満ならスリープ、スロットル上昇のまま
while cs.alt < 50:
    Script.Sleep(100)

# スロットルをニュートラルに設定
Script.SendRC(3,1500,True)
# Loiterモードに切り替える
Script.SendRC(5,2000,True)

Script.SendRC(1,2000,True) # roll
Script.Sleep(2000)
Script.SendRC(1,1000,True) # roll
Script.Sleep(2000)
Script.SendRC(1,1500,True)

# 着陸していく
while cs.alt > 0.5:
    if cs.alt > 1:
        Script.SendRC(3,1200,True)
    else:
        Script.SendRC(3,1000,True)
    Script.Sleep(100)

print('Land Completed!')

# メッセージをクリア
cs.messages.Clear()
Script.Sleep(1000)
# スティックディスアーム送信
Script.SendRC(3,1000,False)
Script.SendRC(4,1000,True)
Script.WaitFor('Disarming motors',30000)
# ヨー方向のチャンネルをニュートラルに設定
Script.SendRC(4,1500,True)

print('Disarmed!')