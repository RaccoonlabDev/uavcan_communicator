#!/bin/bash
# Instruction from https://github.com/UAVCAN/libuavcan/tree/legacy-v0#using-in-a-gnulinux-application

cd "$(dirname "$0")"
cd ../uavcan_communicator/libs/libuavcan
mkdir -p build
cd build
rm -r *  # handle case if build directory is not empty bacause it may lead to fail
cmake ..
make
sudo make install
