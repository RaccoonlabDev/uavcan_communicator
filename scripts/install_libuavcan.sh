#!/bin/bash
CRNT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_DIR="$(dirname "$CRNT_DIR")"

set -e

# Install libuavcan
# Instruction from https://github.com/UAVCAN/libuavcan/tree/legacy-v0#using-in-a-gnulinux-application
cd ${REPO_DIR}/libs/libuavcan
mkdir -p build
cd build
cmake ..
make
sudo make install
