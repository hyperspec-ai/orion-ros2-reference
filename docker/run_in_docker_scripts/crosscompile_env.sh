#!/bin/bash

export TARGET_TRIPLE=aarch64-linux-gnu
#mkdir -p /usr/lib/$TARGET_TRIPLE
ln -f -s /opt/sysroot/lib/$TARGET_TRIPLE/libz.so.1 /usr/lib/$TARGET_TRIPLE/libz.so
ln -f -s /opt/sysroot/lib/$TARGET_TRIPLE/libpcre.so.3 /usr/lib/$TARGET_TRIPLE/libpcre.so

export TARGET_ARCH=aarch64
export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
export SYSROOT=/opt/sysroot
export ROS2_INSTALL_PATH=/opt/sysroot/opt/ros2_ws/install
export CMAKE_FIND_ROOT_PATH=/opt/ws_kros/build
export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE
export PWD=/opt/sysroot/opt/ros2_ws
