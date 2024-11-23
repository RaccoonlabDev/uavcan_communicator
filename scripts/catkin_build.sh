#!/bin/bash
# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2022-2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

source /opt/ros/$ROS_DISTRO/setup.bash
cd /catkin_ws
git config --global http.sslverify false
catkin build
