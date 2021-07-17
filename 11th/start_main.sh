#!/usr/bin/bash

AP_HOME=/home/tsuyoshi/GitHub/tajisoft/ardupilot

declare -a loc=("35.79196857,139.0513938,0,0" "35.80005627,139.0842228,0,0" "35.80801586,139.0705794,0,0" "35.81240078,139.0938168,0,0" "35.81800657,139.1495311,0,0")

for i in `seq 1 $1`
do
  session=mav${i}
  mavid=$i
  ins=`expr $i - 1`
  locidx=`echo $(( $i % 5 ))`
  locidx=`expr $locidx`
  myloc=${loc[$locidx]}
  echo "$session :: $mavid at $myloc"
  screen -S $session -dm $AP_HOME/Tools/autotest/sim_vehicle.py -v ArduCopter -I$ins --custom-location=$myloc --sysid $mavid
  sleep 1
done
