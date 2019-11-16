# -*- ecoding: utf-8 -*-
####################################################
# サンプルコード
# スティックアーミングをスクリプトで実装
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