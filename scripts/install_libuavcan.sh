#!/bin/bash
# Instruction from https://github.com/UAVCAN/libuavcan/tree/legacy-v0#using-in-a-gnulinux-application

cd "$(dirname "$0")"
cd ../drone_communicators/libs/libuavcan
mkdir build
cd build
cmake ..
make
sudo make install
