# ArduPilot SITL on Docker
DockerコンテナでArduPilotのビルド実行およびSITL起動するための環境構築手順です。

# 参考文献
- [https://discuss.ardupilot.org/t/ardupilot-and-docker-part-1/90532](https://discuss.ardupilot.org/t/ardupilot-and-docker-part-1/90532)
- [https://github.com/radarku/ardupilot-sitl-docker](https://github.com/radarku/ardupilot-sitl-docker)

# 留意事項
- Mac,Windowsともにビルドが遅いです。

# 手順
Dockerをインストールを必須とします。[https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/)を参考にしてインストールをしてください。<br/>
WindowsのDocker環境は[Docker Desktop](https://www.docker.com/products/docker-desktop/)、ターミナルはPowerShellを前提としています。  
TODO 別ドキュメントにまとめる。

## 1. ArduPilotソースコードをクローン（１回のみ）
（※Windowsの場合のみ）改行コードをLFとする必要があるため、クローン前に下記コマンドを実行する。クローン後、必要に応じて`git config --global core.autocrlf=true`を実行して設定を戻すこと。
```
$ git config --global core.autocrlf=false
```
下記コマンドを実行してサブモジュールを含むソースコードをクローンする。
```
$ git clone https://github.com/ArduPilot/arduilot
$ cd ardupilot
$ git submodule update --init --recursive
```

## 2. コンテナイメージ作成（１回のみ）
```
$ docker build . -t ardupilot
```
![ビルド](ardupilot-docker-build.png)

## 3. コンテナ起動（毎回）
コンテナを起動しコンテナ内に入る。オプションは下記の通り。<br/>
`-p`: ポート設定<br/>
`-v`: マウント設定<br/>
`--rm`: コンテナ削除（任意）
```
[Macの場合]
$ docker run -it -p 14550:14550/udp -v `pwd`:/ardupilot ardupilot:latest bash
[Windowsの場合]
$ docker run -it -p 14550:14550/udp -v ${PWD}:/ardupilot ardupilot:latest bash
※↓このようにプロンプトが変わる。
ardupilot@25b5653ae641:/ardupilot$
```

![コンテナプロンプト](ardupilot-container-prompt.png)

<span style="color:red">【注意】現状MAVProxy.pyのパスを手動でexportコマンドで都度通す必要がある。次のような表示になる。入力するのは`export PATH=$HOME/.local/bin:$PATH`の部分のみ。</span>
```
ardupilot@25b5653ae641:/ardupilot$ export PATH=$HOME/.local/bin:$PATH
```
`sim_vehicle.py`から実行される`git rev-parse`コマンド失敗回避のため、`/ardupilot` をgitのセーフディレクトリに追加する必要がある。
```
ardupilot@25b5653ae641:/ardupilot$ git config --global --add safe.directory /ardupilot
```

## 4. SITL起動（毎回）
例えば、同じネットワーク（192.168.11.0/24）につながっているクライアント（192.168.11.20）にテレメトリーを接続させる場合は次のようなコマンドになります。
```
[Copterの場合]
ardupilot@25b5653ae641:/ardupilot$ sim_vehicle.py -v Copter --out 192.168.11.20:14550
```

## 5. コンテナ停止
コンテナを停止するのは簡単です。exitコマンドを実行するだけでよいです。コンテナを停止した後でも作成したコンテナイメージは削除されません。
```
ardupilot@25b5653ae641:/ardupilot$ exit
```
