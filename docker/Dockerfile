FROM ros:melodic
LABEL description="Uavcan communicator"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/uavcan_communicator

# 1. Install basic requirements
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git python-catkin-tools python-pip python3-pip
RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# 2. Install package requirements
COPY scripts/ scripts/
COPY libs/ libs/
RUN scripts/install_requirements.sh
RUN scripts/install_libuavcan.sh

# 3. Install dependencies
RUN git clone https://github.com/InnopolisAero/uavcan_msgs.git /catkin_ws/src/uavcan_msgs
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

# 4. Copy main package files
COPY CMakeLists.txt     CMakeLists.txt
COPY package.xml        package.xml
COPY src/ src/

# 5. For custom uavcan msgs
# COPY custom_msgs/ custom_msgs/
# RUN ./scripts/compile_dsdl.sh

# 6. Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

# 7. Copy auxilliary package files
COPY config/ config/
COPY launch/ launch/

CMD source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    echo "main process has been started" && \
    roslaunch uavcan_communicator inno_dynamics_communicator.launch && \
    echo "container has been finished"
