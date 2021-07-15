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
    apt-get install -y  git                     \
                        python-catkin-tools     \
                        python-pip              \
                        python3-pip

# 2. Additional
RUN sudo apt-get install -y iputils-ping        \
                            nano                \
                            iproute2            \
                            net-tools           \
                            tcpdump             \
                            nmap

# 3. Setup slcan
RUN sudo apt-get install -y socat               \
                            kmod

# 4. Install uavcan_communicator
COPY drone_communicators/ drone_communicators/
COPY scripts/ scripts/
COPY uavcan_msgs/ uavcan_msgs/
RUN scripts/install_requirements.sh
RUN scripts/install_libuavcan.sh
# RUN ./scripts/compile_dsdl.sh

# 5. Build
RUN source /opt/ros/melodic/setup.bash      &&  \
    cd ../../                               &&  \
    catkin build


CMD source /opt/ros/melodic/setup.bash      && \
    source /catkin_ws/devel/setup.bash      && \
    echo "main process has been started"    && \
    echo "do nothing yet"                   && \
    echo "container has been finished"
