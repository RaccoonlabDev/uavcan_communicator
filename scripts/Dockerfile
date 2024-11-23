# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2022-2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
ARG ROS_DISTRO=noetic
FROM ros:$ROS_DISTRO
LABEL description="DroneCAN communicator"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/dronecan_communicator

# 1. Install requirements
COPY scripts/install.sh scripts/install.sh
RUN ./scripts/install.sh

# 2. Build ROS
COPY . /catkin_ws/src/dronecan_communicator
RUN /catkin_ws/src/dronecan_communicator/scripts/catkin_build.sh

CMD source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    echo "main process has been started" && \
    roslaunch uavcan_communicator example_communicator.launch && \
    echo "container has been finished"
