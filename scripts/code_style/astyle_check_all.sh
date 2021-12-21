#!/bin/bash
# Check http://astyle.sourceforge.net/astyle.html


ASTYLE_SCRIPT_PATH=$(dirname $(readlink -f $0))/astyle_check_single_file.sh
WS_DIR=$(dirname $(readlink -f $0))/../..
SRC_DIR=$WS_DIR/uavcan_communicator/src/uavcan_communicator
cd $WS_DIR

for filename in $SRC_DIR/*; do
   $ASTYLE_SCRIPT_PATH $filename
done

# Delelte temp file and Cause an error if code style is not ok 
if [ -s "temp_astyle_output.txt" ]
then
   echo "Your code does not correspond to the code style."
   rm temp_astyle_output.txt
   exit 1
fi
rm temp_astyle_output.txt