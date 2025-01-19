Day 5の課題

隣接ポイントとセブンイレブンをRoverで往復するプログラム作成

mission plannerを立ち上げ、extra command lineに以下を入れてRoverを起動する
--home 35.879129,140.339683,7,0

rover_severneleven.py
を実行すると、

隣接ポイントとセブンイレブンを往復する

本来は、隣接ポイントにCopterが到着したのを判定してRoverを走らせるが、このプログラムはCopterが隣接ポイントに到着したとして、そのあとのRover部分のみをプログラムしている。
