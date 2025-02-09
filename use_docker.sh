#!/bin/bash

if [ "$1" == "build" ]; then

    docker build -t bt_navigator_live .

elif [ "$1" == "run" ]; then

    xhost +local:root
    docker run --name bt_navigator_live -it --rm  \
        --env="DISPLAY=$DISPLAY" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        bt_navigator_live bash
    xhost -local:root

elif [ "$1" == "devel" ]; then

    xhost +local:root
    docker run --name bt_navigator_live -it --rm \
        --env="DISPLAY=$DISPLAY" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $PWD/bt_monitor:/root/ros2_ws/src/bt_monitor \
        bt_navigator_live bash 
    xhost -local:root

elif [ "$1" == "enter" ]; then

    docker exec -it bt_navigator_live bash

else
    echo "Usage: $0 [build|run|devel]"
fi
