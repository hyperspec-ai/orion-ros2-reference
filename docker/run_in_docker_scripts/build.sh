#!/bin/
#source /opt/sysroot/opt/ros2_ws/install/local_setup.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
bash $DIR/build_x86.sh
bash $DIR/build_arm.sh