#!/bin/bash
set -e

# setup ros2 environment
source "$ROS2_WS/install/setup.bash"
#source "$KROS_WS/install/local_setup.bash"
#source "$KROS_WS/install/setup.bash"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$KROS_WS/src/config_node/data/lib_message
cd $KROS_WS
$@
