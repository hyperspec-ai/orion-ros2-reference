#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $SCRIPT_DIR/crosscompile_env.sh

cd /opt/ws_kros
rm -R build
rm -R install_arm

colcon build \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
        -DCMAKE_TOOLCHAIN_FILE="/opt/sysroot/opt/ros2_ws/src/ros2/cross_compile/cmake-toolchains/generic_linux.cmake" \
        -DSECURITY=ON \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_FLAGS="-Wall"
mv /opt/ws_kros/install /opt/ws_kros/install_arm
