#!/bin/bash
cd /opt/ws_kros
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
export SYSROOT=/opt/sysroot
export ROS2_INSTALL_PATH=/opt/sysroot/opt/ros2_ws/install
export CMAKE_FIND_ROOT_PATH=/opt/ws_kros/build
export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE
export PWD=/opt/sysroot/opt/ros2_ws
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