#!/bin/bash

if [ "$1" == "build" ]; then
    docker build -t bt_navigator_live .
elif [ "$1" == "run" ]; then
    docker run -it --rm --name bt_navigator_live --env="DISPLAY" bt_navigator_live bash
elif [ "$1" == "devel" ]; then
    docker run -it --rm --name bt_navigator_live --env="DISPLAY" -v $PWD/live_navigator_plugins:/root/ros2_ws/src/live_navigator_plugins bt_navigator_live bash 
else
    echo "Usage: $0 [build|run|devel]"
fi
