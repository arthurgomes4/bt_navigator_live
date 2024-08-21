ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}

# set frontend non interactive
ARG DEBIAN_FRONTEND=noninteractive

# install ros2 packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-rviz2

#install Groot with dependencies
RUN apt-get update && apt-get install -y \
    qtbase5-dev \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev

# Clone Groot repository
RUN git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && \
    cmake --build build

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /root/ros2_ws/src/live_navigator_plugins

# COPY live_navigator_plugins /root/ros2_ws/src/live_navigator_plugins

# RUN source /opt/ros/iron/setup.bash && \
#     cd /root/ros2_ws && \
#     colcon build

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc 