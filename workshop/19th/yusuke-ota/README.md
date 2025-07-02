Hello, ArduPilot!

# flight_experience_ota.py

MAVLink対応ドローン向けのPythonスクリプトです。テイクオフ、単一ウェイポイント移動、8の字飛行、RTL（Return to Launch）などをテストできます。

## ✅ 前提条件

- Python 3.9以降
- pymavlink
- geopy

## 🔧 インストール方法

以下のコマンドで依存パッケージをインストールできます：

```bash
pip install --user -r requirements.txt
```

## 🚀 使い方

```bash
python flight_experience_ota.py --connect tcp:127.0.0.1:5763
```

- `--connect`：ドローンまたはシミュレータの接続先（例: MAVProxyやSITLのアドレス）

## 🔍 主な機能

- GUIDEDモードへの変更
- ARM / DISARM テスト（`ARMTEST = True` で有効化）
- 指定高度までのテイクオフ
- 現在地からの単一ウェイポイント飛行
- 8の字飛行（辞書形式のウェイポイント列）
- RTL（リターン・トゥ・ローンチ）

## 📁 ファイル構成

- `flight_experience_ota.py`：メインスクリプト
- `requirements.txt`：依存パッケージ一覧

## 📝 備考

- pymavlink使用時、接続先のMAVLinkデバイスは起動しておく必要があります。