#!/bin/bash
CRNT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_DIR="$(dirname "$CRNT_DIR")"

set -e

apt-get update
apt-get install -y git \
                   can-utils \
                   ros-$ROS_DISTRO-catkin \
                   ros-$ROS_DISTRO-mavros-msgs \
                   python3-pip \
                   python3-catkin-tools
