#!/bin/
export TARGET_TRIPLE=aarch64-linux-gnu
mkdir -p /usr/lib/$TARGET_TRIPLE
ln -f -s /opt/sysroot/lib/$TARGET_TRIPLE/libz.so.1 /usr/lib/$TARGET_TRIPLE/libz.so
ln -f -s /opt/sysroot/lib/$TARGET_TRIPLE/libpcre.so.3 /usr/lib/$TARGET_TRIPLE/libpcre.so
#source /opt/sysroot/opt/ros2_ws/install/local_setup.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
bash $DIR/build_x86.sh
bash $DIR/build_arm.sh