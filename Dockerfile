ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}

# set frontend non interactive
ARG DEBIAN_FRONTEND=noninteractive

# install ros2 packages
# RUN apt-get update && apt-get install -y \
#     ros-${ROS_DISTRO}-navigation2 \
#     ros-${ROS_DISTRO}-rviz2
RUN apt-get update

RUN apt-get install -y ros-humble-behaviortree-cpp-v3

#install Groot with dependencies
RUN apt-get update && apt-get install -y \
    qtbase5-dev \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev

# Clone Groot repository
# RUN git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
#     cd Groot && \
#     cmake -S . -B build && \
#     cmake --build build

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /root/ros2_ws/src/bt_monitor

# COPY bt_monitor /root/ros2_ws/src/bt_monitor

# RUN source /opt/ros/humble/setup.bash && \
#     cd /root/ros2_ws && \
#     colcon build

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc 

# Add alias for Groot executable
# RUN echo "alias groot='/Groot/build/groot'" >> /root/.bashrc