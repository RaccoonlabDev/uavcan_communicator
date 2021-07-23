#!/bin/bash
#Usage: ./scripts/compile_dsdl.sh <path_to_custom_msgs>

cd "$(dirname "$0")"
libuavcan_dsdlc $1 -I/usr/local/share/uavcan/dsdl/uavcan
