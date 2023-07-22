#!/bin/bash

WORKDIR=$HOME/github/droneschool/multiple_vehicles

gnome-terminal -- $WORKDIR/start_vehicle.sh 0 quadplane 14551 14552 &
gnome-terminal -- $WORKDIR/start_vehicle.sh 1 copter 14561 14562 &
gnome-terminal -- $WORKDIR/start_vehicle.sh 2 boat 14571 14572 &
gnome-terminal -- $WORKDIR/start_vehicle.sh 3 rover1 14581 14582 &
gnome-terminal -- $WORKDIR/start_vehicle.sh 4 rover2 14591 14592 &
