docker run  -v $WS_NODE:/opt/ws_kros --privileged -v /dev:/dev --rm orion:dev "colcon build" --cmake-args ' -DCMAKE_BUILD_TYPE=Debug' -DCMAKE_CXX_FLAGS="-Werror -Wall"
