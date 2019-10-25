#!/bin/bash

# When invoked on jenkins, WS_KROS is not set, so we default set it to /opt/ws_kros
if [ -z "$WS_KROS" ]; then
    WS_KROS=/opt/ws_kros
fi

docker run  -v $WS_KROS:/opt/ws_kros --privileged -v /dev:/dev --rm orion:dev \
    "colcon build" --cmake-args ' -DCMAKE_BUILD_TYPE=Debug' -DCMAKE_CXX_FLAGS="-Werror -Wall"

# copy and compress binaries
(cd $WS_KROS;./deploy_arch.sh x64)
