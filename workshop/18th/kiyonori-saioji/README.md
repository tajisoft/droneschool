宿題3

pymavlink_connect-hearbeat_armdisarm_takeoff_location-control.py

pymavlink_scripts/の以下を参考に

pm00_connect_heartbeat.py
pm02_arm_disarm.py
pm10_takeoff.py
pm40_location_control.py

connect、arm、takeoffして30m上昇し、北へ100m進み、到着したらLANDモードで着陸する。

ログで、上昇時の高度、現在地と目的地まで差を表示し、目的地に到着したら着陸時の高度を表示する。