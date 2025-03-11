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

elif [ "$1" == "exec" ]; then
    # Check if a command was provided
    if [ -z "$2" ]; then
        echo "Usage: $0 exec <command>"
        exit 1
    fi
    # Execute the command in the container
    docker exec -it bt_navigator_live ${@:2}

else
    echo "Usage: $0 [build|run|devel|enter|exec <command>]"
    echo "Examples:"
    echo "  $0 build              # Build the container"
    echo "  $0 run               # Run the container"
    echo "  $0 devel             # Run with development mounts"
    echo "  $0 enter             # Enter running container"
    echo "  $0 exec <command>    # Run command in container"
    echo "  $0 exec colcon build # Build the workspace"
fi
