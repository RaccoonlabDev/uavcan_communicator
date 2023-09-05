#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo apt-get install -y can-utils ros-$ROS_DISTRO-mavros-msgs
pip install -r $SCRIPT_DIR/requirements.txt
