# -*- ecoding: utf-8 -*-
####################################################
# サンプルコード
# モード切替をスクリプトで実装
####################################################

print('Start Script')

# Stabilize
Script.ChangeMode('Stabilize')
# 5秒スリープ
print('Stabilizeに変更')
Script.Sleep(5000)

# AltHold
Script.ChangeMode('AltHold')
# 5秒スリープ
print('AltHoldに変更')
Script.Sleep(5000)

# Loiter
Script.ChangeMode('Loiter')
# 5秒スリープ
print('Loiterに変更')
Script.Sleep(5000)

# Guided
Script.ChangeMode('Guided')
# 5秒スリープ
print('Guidedに変更')
Script.Sleep(5000)

# Acro
Script.ChangeMode('Acro')
# 5秒スリープ
print('Acroに変更')
Script.Sleep(5000)

# PosHold
Script.ChangeMode('PosHold')
# 5秒スリープ
print('PosHoldに変更')
Script.Sleep(5000)

print('End Script')