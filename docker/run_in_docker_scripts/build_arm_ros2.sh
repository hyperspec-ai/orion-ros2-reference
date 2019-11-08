#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $SCRIPT_DIR/crosscompile_env.sh

cd /opt/sysroot/opt/ros2_ws
rm -R build
rm -R install_arm

#export Eigen3_DIR="/opt/sysroot/opt/ros2_ws/install/eigen3_cmake_module/share/eigen3_cmake_module/cmake/Modules/"

colcon build \
--cmake-force-configure \
--cmake-args \
-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
-DCMAKE_TOOLCHAIN_FILE="$SCRIPT_DIR/xavier-linux-toolchain.cmake" \
-DSECURITY=ON \
-DFORCE_BUILD_VENDOR_PKG=ON \
-DCMAKE_BUILD_TYPE=Debug \
-DCMAKE_CXX_FLAGS="-Wall" \
-DBUILD_TESTING:BOOL=OFF
