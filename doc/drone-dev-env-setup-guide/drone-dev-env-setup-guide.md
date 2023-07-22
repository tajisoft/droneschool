<div align="center">
<h3>ドローンエンジニア養成塾 デベロッパーコース</h3>
<h2>開発環境構築手順書</h2><br>
(Windows10/11 + WSL1(Ubuntu20.04) + Visual Studio Code)<br/>
Ver.1.4.2 - 2023.7.11
</div>

<!--
Ver.1.4.0 - 2023.5.26 - 初版
Ver.1.4.1 - 2023.6.9  - WSLインストール方法変更、PDFレイアウト調整、微修正
Ver.1.4.2 - 2023.7.11 - PDF生成方法修正、画像・コードの改行位置調整
-->

Table of Contents
<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [1. はじめに](#1-はじめに)
- [2. Mission Planner セットアップ](#2-mission-planner-セットアップ)
  - [2.1. Mission Planner インストール](#21-mission-planner-インストール)
  - [2.2. シミュレータ（Mission Planner）の起動](#22-シミュレータmission-plannerの起動)
  - [2.3. Mission Plannerを使用した機体の基本操作](#23-mission-plannerを使用した機体の基本操作)
    - [2.3.1. アームと離陸](#231-アームと離陸)
    - [2.3.2. ミッション作成と自動飛行](#232-ミッション作成と自動飛行)
- [3. Visual Studio Codeインストール](#3-visual-studio-codeインストール)
- [4. WSLにUbuntuをインストール](#4-wslにubuntuをインストール)
  - [4.1. WSLの有効化設定とバージョン確認](#41-wslの有効化設定とバージョン確認)
  - [4.2. Ubuntu20のインストールと初期設定](#42-ubuntu20のインストールと初期設定)
- [5. Visual Studio CodeとWSLの連携](#5-visual-studio-codeとwslの連携)
  - [5.1. 拡張機能のインストール](#51-拡張機能のインストール)
  - [5.2. 日本語表示の有効化](#52-日本語表示の有効化)
  - [5.3. WSLとの接続](#53-wslとの接続)
- [6. ArduPilotビルド環境セットアップ](#6-ardupilotビルド環境セットアップ)
  - [6.1. ArduPilotソースコードをクローン](#61-ardupilotソースコードをクローン)
  - [6.2. セットアップスクリプトで環境をインストール](#62-セットアップスクリプトで環境をインストール)
- [7. WSL（Ubuntu20）へ拡張機能のインストール](#7-wslubuntu20へ拡張機能のインストール)
- [8. シミュレータ（SITL）用セットアップ](#8-シミュレータsitl用セットアップ)
  - [8.1. GUI表示のためのセットアップ](#81-gui表示のためのセットアップ)
  - [8.2. シミュレータ動作確認](#82-シミュレータ動作確認)
- [9. シミュレータ（Gazebo）用セットアップ（任意）](#9-シミュレータgazebo用セットアップ任意)
  - [9.1. Gazeboのインストール](#91-gazeboのインストール)
  - [9.2. プラグインのインストール](#92-プラグインのインストール)
  - [9.3. シミュレータの起動](#93-シミュレータの起動)
- [10. 【Applicationコース向け】DroneKit Python, pymavlinkセットアップ](#10-applicationコース向けdronekit-python-pymavlinkセットアップ)
  - [10.1. DroneKit Python最新のソースコードからインストール](#101-dronekit-python最新のソースコードからインストール)
  - [10.2. pymavlinkソースコードの取得](#102-pymavlinkソースコードの取得)
  - [10.3. 自動補完セットアップ](#103-自動補完セットアップ)
  - [10.4. 動作確認](#104-動作確認)
- [11. 【FlightCodeコース向け】デバッグ環境セットアップ](#11-flightcodeコース向けデバッグ環境セットアップ)
  - [11.1. 必要なパッケージインストール](#111-必要なパッケージインストール)
  - [11.2. デバッグ構成を追加](#112-デバッグ構成を追加)
  - [11.3. ブレークポイントを置く](#113-ブレークポイントを置く)
  - [11.4. デバッグ実行](#114-デバッグ実行)
- [12. Appendix](#12-appendix)
  - [12.1. Visual Studio Codeショートカットキー](#121-visual-studio-codeショートカットキー)

<!-- /code_chunk_output -->

<div style="page-break-before:always"></div>

# 1. はじめに
本書はWindows10/11 + WSL1(Ubuntu20.04) + Visual Studio Code を使用してArduPilotドローンソフトウェアの開発・テストを行うための環境構築手順です。  
開発環境構成における本書の対象範囲は下図の通りとなります。  
![Alt text](media/intro-010.jpg)  

<div style="page-break-before:always"></div>  

# 2. Mission Planner セットアップ
## 2.1. Mission Planner インストール
【注意】インストール済みの場合はスキップしてください。  

下記リンクをクリックして最新のインストーラをダウンロードします。  
https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msi  

ダウンロードされたインストーラ `MissionPlanner-latest.msi` をダブルクリックします。  

ライセンスに同意して、基本的に`Next`、`Install`、`次へ`、`完了` を押下してインストールを完了させます。  

タスクバーに`Mission Planner`を入力して、Mission Plannerを起動します。  

Altitude Angelダイアログが表示されたら、`No`を押下します。（初回のみ）  

## 2.2. シミュレータ（Mission Planner）の起動
上部メニューの `シミュレーション` を押下して、シミュレーション画面に切り替えます。  
![Alt text](media/mp-setup-010.jpg)  

ホームポジションを指定するため、Extra command line に `--home 35.879129,140.339683,7,0` （ドローンフィールドKawachiの場所） を入力して、画面下部の `Multirotor` のアイコン を押下します。   
![Alt text](media/mp-setup-020.jpg)  

Select your versionダイアログが表示されたら、`Stable` を押下します。  
![Alt text](media/mp-setup-030.jpg)  

シミュレータの実行ファイルがダウンロードが開始され、完了後に起動します。
起動後、フライト・データ画面に切り替わり、機体（シミュレータ）に接続されます。  
![Alt text](media/mp-setup-040.jpg)  

シミュレータを終了するには、タイトルバーに `ArduCopter.exe` が表示されているターミナルのウィンドウを閉じます。  
![Alt text](media/mp-setup-050.jpg)  

二回目以降は`Skip Download` にチェックを付けることで、実行ファイルのダウンロードをスキップしてシミュレータを起動できます。  
![Alt text](media/mp-setup-060.jpg)  

<div style="page-break-before:always"></div>  

## 2.3. Mission Plannerを使用した機体の基本操作
【注意】この手順は機体（シミュレータ）に接続している状態で行ってください。  
### 2.3.1. アームと離陸
上部メニューの `フライト・データ` を押下してフライト・データ画面に切り替えます。  
![Alt text](media/mp-setup-070.jpg)  

アクションタブを開き、リストボックスから `Guided` を選択して、 `モードをセット` ボタンを押下して、機体のモードをGUIDEDモードに切り替えます。`Arm/Disarm` ボタンを押下して、機体をアームします。  
![Alt text](media/mp-setup-080.jpg)  
HUDに下記の通り、`ARMED` , `Guided` が表示されていることを確認します。  
![Alt text](media/mp-setup-081.jpg)  

マップエリアで右クリック → `離陸` を押下 → Enter Altダイアログに `10` を入力して、`OK` を押下します。  
![Alt text](media/mp-setup-090.jpg)
![Alt text](media/mp-setup-100.jpg)

機体が離陸し、高度10メートルまで上昇してホバリングすることを確認します。  
![Alt text](media/mp-setup-110.jpg)  

<div style="page-break-before:always"></div>  

目的地を指定した自動飛行を行う場合、マップエリアの任意の場所で右クリック → `ここまで飛行` を押下 →  Enter Altダイアログに `10` を入力して、`OK` を押下します。  
![Alt text](media/mp-setup-120.jpg)
![Alt text](media/mp-setup-130.jpg)  

指定した地点への自動飛行が開始されます。目的地に到達すると機体はホバリングします。  
![Alt text](media/mp-setup-131.jpg)  

その場で着陸する場合は、リストボックスから `Land` を選択して、 `モードをセット` ボタンを押下します。ホームポジションに戻って着陸する場合は、`RTL` ボタンを押下します。  

<div style="page-break-before:always"></div>  

### 2.3.2. ミッション作成と自動飛行
【注意】この手順は機体（シミュレータ）がホバリングしている状態で行ってください。  

上部メニューの `フライト・プラン` を押下してフライト・プラン画面に切り替えます。  
![Alt text](media/mp-setup-140.jpg)  

デフォルト高度に`10`を入力します。マップエリアを左クリックしてウェイポイントを追加してミッションを作成します。`WPの書込み` ボタンを押下して、作成したミッションを機体にアップロードします。  
![Alt text](media/mp-setup-150.jpg)  

上部メニューの `フライト・データ` を押下してフライト・データ画面に切り替えます。  
![Alt text](media/mp-setup-070.jpg)  

<div style="page-break-before:always"></div>  

アクションタブを開き、`Auto` ボタンを押下して、AUTOモードに切り替えます。  
![Alt text](media/mp-setup-160.jpg)  

作成したミッションに基づいた自動飛行が開始されます。ミッションが完了すると機体はホバリングします。  
![Alt text](media/mp-setup-170.jpg)  

ミッションを再実行する場合は、LOITER, GUIDEDなど別のモードに切り替えた後、AUTOモードに切り替えます。  

<div style="page-break-before:always"></div>  

# 3. Visual Studio Codeインストール
【注意】インストール済みの場合はスキップしてください。  

下記サイトを開きます。  
https://code.visualstudio.com/  
`Download for Windows Stable Build` をクリックするとダウンロードが開始されます。  
ダウンロードされた exeファイル `VSCodeUserSetup-x64-＜バージョン番号＞.exe` をダブルクリックしてインストールを進めてください。

<div style="page-break-before:always"></div>  

基本的に `次へ` 、 `インストール` をクリックしてインストールを進めます。下記の画面では `PATHへの追加` を選択してください。  

![Alt text](media/vsc-install-010.jpg)  

Visual Studio Codeのインストールが完了したらPCを再起動して次のステップに進みます。

<div style="page-break-before:always"></div>

# 4. WSLにUbuntuをインストール
## 4.1. WSLの有効化設定とバージョン確認
あまり古いバージョンのWindows10だとWSL機能が使えないため念の為バージョンを確認してください。  
確認するためにPowerShellで `winver` を実行して確認してください。 
```powershell
winver
```
下記の画面が表示されます。  
![Alt text](media/wsl-install-010.jpg)  

WSLをインストールするためには、Windows 10 version 2004(Build 19041)以上、もしくはWindows 11である必要があります。古い場合はWindows10の更新、またはWindows 11のインストールを先に完了してから再度このステップから実行してください。  
社用PCなどセキュリティ対策が施されている場合、仮想化機能が無効化されている場合はセットアップが失敗する可能性があります。自社のテクニカルサポート部門にお問合せください。<br/>
WSLのインストールが可能な条件を満たしている場合次のページのインストールに進んでください。

WSL有効化をするために、PowerShellを管理者権限で開きます。  
タスクバーの検索窓に `PowerShell` を入力して検索します。  
![Alt text](media/wsl-install-011.jpg)  

<div style="page-break-before:always"></div>

検索結果に表示された `Windows PowerShell` の右側にある `>` ボタンをクリックし、`管理者として実行する` をクリックします。  

![Alt text](media/wsl-install-020.jpg)  

立ち上がったPowerShellのウィンドウに次の2つのコマンドを順番に実行し、PCを再起動してください。
```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```
```powershell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

PC再起動後、PowerShellを開いて次のコマンドを実行しPowerShellを閉じてください。これでデフォルトのバージョンを1にします。  
デフォルトが1で困る場合は、WSLバージョン2でインストールした後で個別に `wsl --set-version Ubuntu-20.04 1` を実行して変更してください。
```powershell
wsl --set-default-version 1
```
## 4.2. Ubuntu20のインストールと初期設定
【注意】すでにUbuntu 20.04がインストール済みの場合はスキップしてください。

PowerShellを開いて次のコマンドを実行して`Ubuntu 20.04`をインストールしてください。
```powershell
wsl --install -d Ubuntu-20.04
```

インストールが完了したら `PCを再起動` をしてください。<br/>
PC起動後の初回起動時 `Installing, this may take a few minutes…` としばらく表示されます。フリーズではないので、そのままインストールが完了するまで待ちます。  
インストールが終わると username と password をきかれるので、下記の通り入力して設定します。必ず半角英字のみで設定します。

* username : `ardupilot`
* password : `ardupilot` 

パスワードは入力してもセキュリティ上表示されませんが入力されています。間違えた場合はバックスペースで消せます。
画像のようになればUbuntuのインストールは完了です。このウィンドウを閉じます。  
![Alt text](media/ubuntu-setup-030.jpg)  

念の為、WSLのバージョンを確認します。PowerShellを起動し次のコマンドを実行して確認してください。
```powershell
wsl -l -v
```
VERSIONのところが `1` と表示されていれば問題ありません。
```powershell
  NAME                   STATE           VERSION
* Ubuntu-20.04           Running         1
```

> **Note**
> WSLはバージョン1と2を混在させることができます。WSL2はUSB接続を可能にする手順が複雑なため、実機デバイスなどを接続する場合WSL1を推奨します。シミュレータしか使用しない場合はWSL2の方が処理速度が速いのでオススメです。

Ubuntuのインストールが完了したら、Visual Studio Codeと連携するため次のステップに進んでください。  

<div style="page-break-before:always"></div>

# 5. Visual Studio CodeとWSLの連携
【注意】すでに連携済みの場合はスキップしてください。  

Visual Studio Codeを起動します。初回起動時に次のような警告ウィンドウが表示される場合 `アクセスを許可する` を選択してください。  
![Alt text](media/vsc-wsl-link-010.jpg)

Visual Studio Codeが起動したら次の手順に進んでください。
## 5.1. 拡張機能のインストール
左側の `Extensions（ブロックのようなアイコン）` を選択し拡張機能をインストールします。検索欄に `remote dev` を入力し検索し `Remote Development` を選択します。詳細画面にある `Install` を選択してください。  
![Alt text](media/vsc-wsl-link-020.jpg)

<div style="page-break-before:always"></div>

同様の手順で次の拡張機能もインストールしてください。

|Extentions|検索ワード|japanese|
|----|----|----|
|Japanese Language Pack for Visual Studio Code|japanese|表示の日本語化|
|Python|Python|Python言語サポート|
|C/C++|C++|C/C++言語サポート|
|Lua|Lua|Lua言語サポート|
|Docker|Docker|Dockerサポート|
|ardupilot-devenv|ardupilot|ArduPilot開発サポート|
|Lua Autocomplete for ArduPilot|ardupilot|ArduPilot用Lua言語サポート|

## 5.2. 日本語表示の有効化
`Ctrl + Shift + P` を押して表示される入力エリアに `Configure Di` と入力します。  
表示される候補の中から `Configure Display Language` を選択します。  
![Alt text](media/vsc-wsl-link-021.jpg)

その後に表示される候補から `日本語(ja)` を選択します。   
![Alt text](media/vsc-wsl-link-022.jpg)


Visual Studio Codeを再起動を促されるので再起動します。起動したら次の手順に進んでください。 
## 5.3. WSLとの接続
画像のように左側の `リモートエクスプローラー（PC画面のようなアイコン）` を選択します。  
画面が変わったら、 `リモートエクスプローラー` を `WSLターゲット` に変更します。画像のようにインストールした `Ubuntu-20.04` が表示されているか確認します。WSLに詳しい方はこの通りでなくても問題ありませんのでビルド環境セットアップに進んでください。  
![Alt text](media/vsc-wsl-link-030.jpg)  

`Ubuntu-20.04` を右クリックし `既定のディストリビューションとして設定` を選択しデフォルトに設定します。  
![Alt text](media/vsc-wsl-link-040.jpg)

設定したら次は、同じ右クリックメニューの `新しいウィンドウで接続する` でWSLに接続します。
新しいウィンドウが開き、左下の部分が画像のような接続した状態になっていることを確認します。  
![Alt text](media/vsc-wsl-link-050.jpg)

<div style="page-break-before:always"></div>

# 6. ArduPilotビルド環境セットアップ
【注意】すでにビルド環境がセットアップ済みの場合スキップしてください。  
## 6.1. ArduPilotソースコードをクローン
Ubuntu20.04を起動します。
コース毎にクローンするURLを確認します。

【Applicationコース向け】本家リポジトリURL  
  https://github.com/ArduPilot/ardupilot.git

【FlightCodeコース向け】Githubアカウントを作成し、本家ardupilotリポジトリをフォークしてから、自分のアカウントのardupilotリポジトリをクローンするのがよいです。その場合のURLは、  
  https://github.com/[自分のGithubアカウント名]/ardupilot.git  
になるはずです。

次のようなコマンドを入力してクローンを実行します。  
※ここでは、Applicationコース向けの本家リポジトリURLを使い、クローン先はhomeディレクトリ `/home/ardupilot` としています。
```bash
cd
```
```bash
git clone https://github.com/ArduPilot/ardupilot.git
```
![Alt text](media/ardupilot-setup-010.jpg)  
クローンが完了したら次の環境セットアップスクリプトを実行します。  

<div style="page-break-before:always"></div>

## 6.2. セットアップスクリプトで環境をインストール
Ubuntu端末に次のコマンドを順番に実行してビルド環境をインストール＆セットアップしていきます。  
```bash
cd ardupilot
```
```bash
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```
何度かパスワードを要求されるので都度入力します。結構処理に時間がかかるので待ちます。  

<div style="page-break-before:always"></div>

# 7. WSL（Ubuntu20）へ拡張機能のインストール
Visual Studio Codeを起動し、WSL（Ubuntu-20.04）に接続します。  
メニューを `ファイル` -> `フォルダーを開く` の順に選択し、前項でダウンロードしたardupilotディレクトリ（パス：`/home/ardupilot/ardupilot`）を開きます。  
![Alt text](media/vsc-ext-install-010.jpg)  

初回は信頼ダイアログが表示されるので `はい、作成者を信頼します` を選択します。  
![Alt text](media/ardupilot-setup-020.jpg)

[Visual Studio CodeとWSLの連携](#5-visual-studio-codeとwslの連携) の手順を参考にして下記の拡張機能をインストールします。

|Extentions|検索ワード|japanese|
|----|----|----|
|Python|Python|Python言語サポート|
|C/C++|C++|C/C++言語サポート|
|Lua|Lua|Lua言語サポート|
|Docker|Docker|Dockerサポート|
|ardupilot-devenv|ardupilot|ArduPilot開発サポート|
|Lua Autocomplete for ArduPilot|ardupilot|ArduPilot用Lua言語サポート|

<div style="page-break-before:always"></div>

# 8. シミュレータ（SITL）用セットアップ
## 8.1. GUI表示のためのセットアップ
【注意】すでにセットアップ済みの場合はスキップしてください。

下記URLから必要なアプリをダウンロードします。  
https://sourceforge.net/projects/vcxsrv/

`Download` ボタンを押下し `VcXsrv Windows X Server` をダウンロードしてからインストールします。
インストールは、`Next` -> `Install` -> `インストール処理` -> `Close` と選択していけばできます。

インストール完了後、スタートボタンで `XLaunch`  と検索しXLaunchアプリを起動します。  
![Alt text](media/SITL-setup-010.jpg)  

デフォルトのままで「次へ」を選択してください。  
![Alt text](media/SITL-setup-020.jpg)  

`Disable access control` にチェックを忘れないようにします。  
![Alt text](media/SITL-setup-030.jpg)  

自動起動設定のために `Save configuration` を選択し設定ファイルを一旦任意の場所に保存してください。`完了` を選択してください。  
![Alt text](media/SITL-setup-040.jpg)  

セキュリティダイアログが表示されたら、パブリックネットワークも必ずチェックしてから `アクセスを許可する` を選択してください。  
![Alt text](media/SITL-setup-050.jpg)  

キーボードのWinキー＋rを押して `ファイル名を指定して実行` ウィンドウを開き、次の文字列を入力して `OK` を選択してください。
```
shell:startup
```

前のステップで保存したXLaunchの設定ファイルを開いたフォルダに移動します。  
![Alt text](media/SITL-setup-060.jpg)  

次に、Ubuntu20.04 を起動して、次のコマンドを実行してから閉じてください。
```bash
echo "export DISPLAY=localhost:0.0" >> ~/.bashrc
```

<div style="page-break-before:always"></div>

コマンド実行後、次のコマンドでファイルに正しく書き込まれていることを確認します。
```bash
cat ~/.bashrc
```

下記のような設定が2行追加されていることを確認します。追加されていない場合は手動で修正してください。
```bash
source /mnt/c/dev/drone-dev/ardupilot/Tools/completion/completion.bash
export DISPLAY=localhost:0.0
```

## 8.2. シミュレータ動作確認
Ubuntu端末を閉じ、PCを再起動してください。
再起動後、タスクトレイに画像のようなアイコンが表示されていることを確認してください。  
![Alt text](media/SITL-setup-070.jpg)  

Ubuntu20.04を起動して次のコマンドを実行してください。
```bash
sim_vehicle.py -v Copter --map --console -L Kawachi
```

セキュリティ警告が表示されたら「アクセスを許可する」を選択してください。  
![Alt text](media/SITL-setup-080.jpg)

画像のような表示になれば正しく動作している状態です。  
![Alt text](media/SITL-setup-090.jpg)  

シミュレータが起動している状態でMission Plannerを起動すると自動的にUDPでシミュレータに接続します。  
![Alt text](media/SITL-setup-100.jpg)  

<div style="page-break-before:always"></div>

# 9. シミュレータ（Gazebo）用セットアップ（任意）
【注意】スペックが低いマシンの場合はGazeboの表示品質が悪くなります。WSL1はGPU非対応のため、GPU有無は表示品質には影響しません。  
【注意】環境によってトラブルが多いため任意のセットアップです。
## 9.1. Gazeboのインストール
Ubuntu20.04 を起動して、下記コマンドを実行してパッケージを更新してください。パスワードを聞かれたら前の手順で設定したパスワードを入力してください。  
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" → /etc/apt/sources.list.d/gazebo-stable.list'
```
```bash
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
```bash
sudo apt update
```

下記コマンドを実行してGazeboをインストールしてください。
```bash
sudo apt install gazebo9 libgazebo9-dev
```

<div style="page-break-before:always"></div>

下記コマンドを実行してGazeboが起動することを確認してください。  
Windows Defender ファイアウォールの確認ダイアログが表示されたら `アクセスを許可する` を選択してください。
```bash
gazebo --verbose
```
![Alt text](media/gazebo-setup-010.jpg)

起動が確認できたら、 `Ctrl + c` でGazeboを終了します。

## 9.2. プラグインのインストール
Ubuntu20.04 に下記コマンドを実行してプラグインのソースコード取得とビルドを行ってください。
```bash
sudo apt install -y cmake
```
```bash
git clone https://github.com/khancyr/ardupilot_gazebo
```
```bash
cd ardupilot_gazebo
```
```bash
mkdir build
```
```bash
cd build
```
```bash
cmake ..
```
```bash
make -j4
```
```bash
sudo make install
```

下記エラーが発生する場合があります。解決策を参考にしてエラーを解消してください。

エラー内容：
```bash
gzclient: error while loading shared libraries: libQt5Core.so.5: cannot open shared object file: No such file or directory
```
解決策コマンド：
```bash
sudo strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
```
参考URL： https://usagi.hatenablog.jp/entry/2019/12/26/181101  

エラー内容：
```bash
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
```
解決策コマンド：
```bash
echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc
```
参考URL： https://volvox.hateblo.jp/entry/2019/01/06/205054 

## 9.3. シミュレータの起動
別のUbuntuウィンドウを開いて下記コマンドを実行してSITLを起動してください。
```bash
sim_vehicle.py -f gazebo-iris -v Copter --console –map -L Kawachi
```

<div style="page-break-before:always"></div>

下記コマンドを実行してGazeboを起動してください。`ardupilot_gazebo` は前の手順で作業したディレクトリです。
```bash
cd ardupilot_gazebo
```
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```


SITLを起動したウィンドウで下記コマンドを実行してください。
```bash
mode guided
rc 3 1000
arm throttle
takeoff 10
```

SITLと同期してGazebo上の機体が離陸することが確認できます。
![Alt text](media/gazebo-setup-020.jpg)

次のページ以降は、コースによって必要なところだけセットアップしてください。  


* [【Applicationコース向け】DroneKit Python, pymavlinkセットアップ](#9-applicationコース向けdronekit-python-pymavlinkセットアップ)  
* [【FlightCodeコース向け】デバッグ環境セットアップ](#10-flightcodeコース向けデバッグ環境セットアップ)

<div style="page-break-before:always"></div>

# 10. 【Applicationコース向け】DroneKit Python, pymavlinkセットアップ
【注意】セットアップ済みの場合はスキップしてください。
## 10.1. DroneKit Python最新のソースコードからインストール
シミュレータ、Ubuntu20.04、Visual Studio Codeを全部終了してください。次にVisual Studio Codeを起動しWSLに接続します。

メニュー `ターミナル` → `新しいターミナル` を選択します。
ターミナルタブに次のコマンドを順番に実行してください。
```bash
cd
```
```bash
git clone https://github.com/dronekit/dronekit-python
```
```bash
cd dronekit-python
```
```bash
pip3 install . --user
```
下記のような実行結果になれば正常にインストールが完了しています。  
```bash
Requirement already satisfied: monotonic>=1.3 in /home/ardupilot/.local/lib/
～省略～
Successfully built dronekit
Installing collected packages: dronekit
  Attempting uninstall: dronekit
    Found existing installation: dronekit 2.9.2
    Uninstalling dronekit-2.9.2:
      Successfully uninstalled dronekit-2.9.2
Successfully installed dronekit-2.9.2
```

<div style="page-break-before:always"></div>

## 10.2. pymavlinkソースコードの取得
自動補完用にpymavlinkのソースコードを取得します。ターミナルタブに次のコマンドを入力して実行してください。
```bash
cd
```
```bash
git clone https://github.com/ArduPilot/pymavlink
```
次のステップに進んで、自動補完のセットアップをしてください。

## 10.3. 自動補完セットアップ
メニュー `ファイル` → `ユーザ設定` → `設定` の順に選択します。下記画面右上の設定ファイルアイコンをクリックします。  
![Alt text](media/dev-app-setup-010.jpg)  

<div style="page-break-before:always"></div>

下記の設定を追加します。既存の設定がすでにある場合はご自身の環境に合わせて設定してください。  
※ DroneKit Python, pymavlink のクローン先が `/home/ardupilot` の場合です。異なる場合は適宜修正してください。
```json
{
    "python.autoComplete.extraPaths": [
        "/home/ardupilot/pymavlink",
        "/home/ardupilot/dronekit-python"
    ],
    "python.analysis.extraPaths": [
        "/home/ardupilot/pymavlink",
        "/home/ardupilot/dronekit-python"
    ],
    ～省略～
}
```

## 10.4. 動作確認
自動補完の動作確認をするために `ファイル` → `新規ファイル` を選択してください。  
![Alt text](media/dev-app-setup-020.jpg)  

次に、新規ファイルウィンドウ部分にフォーカスし `Ctrl + s` で保存メニューを表示してください。`/home/ardupilot/test.py` となるように保存します。   
![Alt text](media/dev-app-setup-030.jpg)  

画像のようにソースコードを入力した際に補完候補が表示されるようになっていればセットアップ完了です。  
* DroneKit Python  
 ![Alt text](media/dev-app-setup-040.jpg)  

* pymavlink  
![Alt text](media/dev-app-setup-050.jpg)  

<div style="page-break-before:always"></div>

# 11. 【FlightCodeコース向け】デバッグ環境セットアップ
【注意】セットアップ済みの場合はスキップしてください。
## 11.1. 必要なパッケージインストール
Ubuntu20.04を起動し次のコマンドを実行してください。  
```bash
sudo apt install gdb -y
```
## 11.2. デバッグ構成を追加
ArduPilotのソースコードを開きます。メニュー `ファイル` → `フォルダーを開く…` → `/home/ardupilot/ardupilot`と入力 を選択してください。  
任意のcppファイルを開いている状態で、メニュー `実行` → `構成の追加…` を選択してください。  
![Alt text](media/fc-debug-setup-010.jpg)  

<div style="page-break-before:always"></div>

表示された `launch.json` の右下 `構成の追加` ボタンをクリックします。
追加された構成を次のように修正して保存ください。 
```json
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) ArduCopter",
            "type": "cppdbg",
            "request": "attach",
            "program": "${workspaceFolder}/build/sitl/bin/arducopter",
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "gdb の再フォーマットを有効にする",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description": "逆アセンブリ　フレーバーを Intel に設定",
                    "text": "-gdb-set disassembly-flavor intel",
                    "ignoreFailures": true
                }
            ]
        }
    ] 
```

<div style="page-break-before:always"></div>

## 11.3. ブレークポイントを置く
一般的なデバッグ手法のやり方として、任意の処理行で一時停止するためのブレークポイントを配置することができます。ブレークポイントは複数設定できます。

ここでは例として、`ArduCopter/mode_stabilize.cpp` のソースコードファイルを開き、画像のように行番号の左側をクリックしてブレークポイントを配置してください。  
![Alt text](media/fc-debug-setup-020.jpg)  

<div style="page-break-before:always"></div>

## 11.4. デバッグ実行
Visual Studio Codeのターミナルから次のコマンドを1度だけ実行してください。
デバッグのたびに実行する必要はありません。ただし、再起動時は再度実行する必要があります。  
```bash
echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
```

次にシミュレータを次のコマンドで起動します。  
```bash
sim_vehicle.py -v Copter --console --map -D -L Kawachi
```
![Alt text](media/fc-debug-setup-030.jpg)  
![Alt text](media/fc-debug-setup-040.jpg)  

メニュー `実行` → `デバッグの開始` を選択してください。  
![Alt text](media/fc-debug-setup-050.jpg)  

`arducopter` プロセスを選択してください。  
![Alt text](media/fc-debug-setup-060.jpg)  

プロセスにアタッチされ、デバッグアイコンメニュー画面は画像のような表示になります。
![Alt text](media/fc-debug-setup-070.jpg)  

GDBを利用したデバッグについて知りたい場合は、下記を参照してください。  
https://ardupilot.org/dev/docs/debugging-with-gdb-using-vscode.html

<div style="page-break-before:always"></div>

# 12. Appendix
## 12.1. Visual Studio Codeショートカットキー
英語：https://code.visualstudio.com/shortcuts/keyboard-shortcuts-windows.pdf
