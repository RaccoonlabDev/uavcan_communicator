ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="DroneCAN communicator"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/dronecan_communicator

# 1. Install basic requirements
RUN apt-get update && apt-get upgrade -y && apt-get install -y git ros-$ROS_DISTRO-catkin python3-pip python3-catkin-tools
RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# 2. Install package requirements
COPY scripts/ scripts/
COPY libs/ libs/
RUN scripts/install_requirements.sh
RUN scripts/install_libuavcan.sh

# 3. Copy main package files
COPY CMakeLists.txt     CMakeLists.txt
COPY package.xml        package.xml
COPY src/ src/

# 4. For custom DroneCAN msgs
# COPY custom_msgs/ custom_msgs/
# RUN ./scripts/compile_dsdl.sh

# 5. Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

# 6. Copy auxilliary package files
COPY config/ config/
COPY launch/ launch/

CMD source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    echo "main process has been started" && \
    roslaunch uavcan_communicator inno_dynamics_communicator.launch && \
    echo "container has been finished"
