FROM ros:melodic
LABEL description="Uavcan communicator"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/drone_communicators

# Install basic requirements
RUN apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git                     \
                        iproute2                \
                        ros-melodic-catkin      \
                        python-catkin-tools     \
                        net-tools               \
                        tcpdump                 \
                        python-pip

# Install this package requirements
COPY . .
RUN chmod +x install_requirements.sh            \
             install_libuavcan.sh           &&  \
    ./install_requirements.sh               &&  \
    ./install_libuavcan.sh

# Build
RUN source /opt/ros/melodic/setup.bash      && \
    cd ../../                               && \
    catkin build                            && \
    source devel/setup.bash                 && \
    roscd drone_communicators

CMD source /opt/ros/melodic/setup.bash      && \
    source /catkin_ws/devel/setup.bash      && \
    echo "main process has been started"    && \
    echo "do nothing yet"                   && \
    echo "container has been finished"
