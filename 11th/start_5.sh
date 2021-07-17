#!/usr/bin/bash

AP_HOME=/home/tsuyoshi/GitHub/tajisoft/ardupilot

screen -S mav1 -dm $AP_HOME/Tools/autotest/sim_vehicle.py -v ArduCopter -I0 --custom-location=35.79196857,139.0513938,0,0 --sysid 1
sleep 1
screen -S mav2 -dm $AP_HOME/Tools/autotest/sim_vehicle.py -v ArduCopter -I1 --custom-location=35.80005627,139.0842228,0,0 --sysid 2
sleep 1
screen -S mav3 -dm $AP_HOME/Tools/autotest/sim_vehicle.py -v ArduCopter -I2 --custom-location=35.80801586,139.0705794,0,0 --sysid 3
sleep 1
screen -S mav4 -dm $AP_HOME/Tools/autotest/sim_vehicle.py -v ArduCopter -I3 --custom-location=35.81240078,139.0938168,0,0 --sysid 4
sleep 1
screen -S mav5 -dm $AP_HOME/Tools/autotest/sim_vehicle.py -v ArduCopter -I4 --custom-location=35.81800657,139.1495311,0,0 --sysid 5

