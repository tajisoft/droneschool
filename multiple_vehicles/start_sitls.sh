#!/bin/bash

# Copterの処理
function start_copter() {
    echo "Starting Copter $1"
    sim_vehicle.py -v Copter --console --map -I$1 $2
}

# Roverの処理
function start_rover() {
    echo "Starting Rover $1"
    sim_vehicle.py -v Rover --console --map -I$1 $2
}

# Boatの処理
function start_boat() {
    echo "Starting Boat $1"
    sim_vehicle.py -v Rover --frame motorbaot --console --map -I$1 $2
}

# Quadplaneの処理
function start_quadplane() {
    echo "Starting Quadplane $1"
    sim_vehicle.py -v Plane --frame quadplane --console --map -I$1 $2
}

# Planeの処理
function start_plane() {
    echo "Starting Plane $1..."
    sim_vehicle.py -v Plane --console --map -I$1 $2
}

# 引数の数をチェック
if [ $# -ne 2 ]; then
    echo "Usage: $0 <instance_number> <frame_type>"
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
        start_copter $1 $2
        ;;
    rover)
        start_rover $1 $2
        ;;
    boat)
        start_boat $1 $2
        ;;
    quadplane)
        start_quadplane $1 $2
        ;;
    plane)
        start_plane $1 $2
        ;;
    *)
        echo "Error: The second argument must be one of 'copter', 'rover', 'quadplane', or 'plane'."
        exit 1
        ;;
esac

exit 0
