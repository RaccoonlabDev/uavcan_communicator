#!/bin/bash
cd "$(dirname "$0")"/../..
pwd
PACKAGE_DIR=uavcan_communicator
SRC_DIR=$PACKAGE_DIR/src/uavcan_communicator
cpplint --root=$PACKAGE_DIR $SRC_DIR/*
