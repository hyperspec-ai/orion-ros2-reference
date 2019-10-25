#!/bin/bash
cd /opt/ws_kros
rm -R build
rm -R install_x86
export ROS2_INSTALL_PATH=/opt/sysroot/opt/ros2_ws/install
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug' -DCMAKE_CXX_FLAGS="-Wall"
mv /opt/ws_kros/install /opt/ws_kros/install_x86