#!/bin/bash

SPEEDUP=10
MP_IP=192.168.11.11 # IP of your pc which running mission planner

# Copterの処理
function start_copter() {
    echo "Starting Copter $1"
    sim_vehicle.py -v Copter --speedup $SPEEDUP -I$1 --out 127.0.0.1:$3 --out $MP_IP:$4 --custom-location 35.878275,140.338069,0,0
}

# Roverの処理
function start_rover1() {
    echo "Starting Rover $1"
    sim_vehicle.py -v Rover --speedup $SPEEDUP -I$1 --out 127.0.0.1:$3 --out $MP_IP:$4 --custom-location 35.879768,140.348495,0,0
}
function start_rover2() {
    echo "Starting Rover $1"
    sim_vehicle.py -v Rover --speedup $SPEEDUP -I$1 --out 127.0.0.1:$3 --out $MP_IP:$4 --custom-location 35.867003,140.305987,0,0
}

# Boatの処理
function start_boat() {
    echo "Starting Boat $1"
    sim_vehicle.py -v Rover --frame motorboat --speedup $SPEEDUP -I$1 --out 127.0.0.1:$3 --out $MP_IP:$4 --custom-location 35.878275,140.338069,0,0
}

# Quadplaneの処理
function start_quadplane() {
    echo "Starting Quadplane $1"
    sim_vehicle.py -v Plane --frame quadplane --speedup $SPEEDUP -I$1 --out 127.0.0.1:$3 --out $MP_IP:$4 --custom-location 35.760215,140.379330,0,0
}

# Planeの処理
function start_plane() {
    echo "Starting Plane $1..."
    sim_vehicle.py -v Plane --speedup $SPEEDUP -I$1 --out 127.0.0.1:$3 --out $MP_IP:$4 --custom-location 35.760215,140.379330,0,0
}

# 引数の数をチェック
if [ $# -ne 4 ]; then
    echo "Usage: $0 <instance_number> <frame_type> <out>"
    exit 1
fi

# 第1引数（数値）のチェック
if ! [[ $1 =~ ^[0-9]+$ ]] || [ $1 -lt 0 ]; then
    echo "Error: The first argument must be a non-negative number."
    exit 1
fi

# 第2引数（文字列）のチェックと関数の呼び出し
case $2 in
    copter)
        start_copter $1 $2 $3 $4
        ;;
    rover1)
        start_rover1 $1 $2 $3 $4
        ;;
    rover2)
        start_rover2 $1 $2 $3 $4
        ;;
    boat)
        start_boat $1 $2 $3 $4
        ;;
    quadplane)
        start_quadplane $1 $2 $3 $4
        ;;
    plane)
        start_plane $1 $2 $3 $4
        ;;
    *)
        echo "Error: The second argument must be one of 'copter', 'rover', 'quadplane', or 'plane'."
        exit 1
        ;;
esac

exit 0
