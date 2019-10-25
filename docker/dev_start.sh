#!/bin/bash

# read start mode from arguments
START_MODE=$1

if [ -z "$START_MODE" ]; then
    START_MODE=shell
fi

DOCKER_BASE_CMD="docker run --ip 172.18.0.10 -it -v $WS_KROS:/opt/ws_kros --privileged -v /dev:/dev --rm orion:dev"
ORION_START_CMD="install/config/lib/config/ConfigNode_exec ./config"

case "$START_MODE" in
    run)
        $DOCKER_BASE_CMD $ORION_START_CMD
        ;;
    debug)
        $DOCKER_BASE_CMD gdbserver :1234 $ORION_START_CMD
        ;;
    shell)
        $DOCKER_BASE_CMD
        ;;
    *)
        echo "Usage: $0 {run|debug|shell}".
    exit 1
esac
