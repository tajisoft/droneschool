#!/usr/bin/bash

AP_HOME=/home/kawamura/GitHub/ardupilot

declare -a loc=("36.0408637,138.1013966,0,0" "36.0408419,138.1014061,0,0" "36.0408284,138.1013826,0,0" "36.0408110,138.1013652,0,0" "36.0407937,138.1013518,0,0")

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
