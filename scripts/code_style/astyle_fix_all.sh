#!/bin/bash
# Check http://astyle.sourceforge.net/astyle.html
 

ASTYLE_SCRIPT_PATH=$(dirname $(readlink -f $0))/astyle_fix_single_file.sh
WS_DIR=$(dirname $(readlink -f $0))/../..
SRC_DIR=$WS_DIR/uavcan_communicator/src/uavcan_communicator
cd $WS_DIR

for filename in $SRC_DIR/*; do
   $ASTYLE_SCRIPT_PATH $filename
done
