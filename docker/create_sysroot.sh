#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

SSH_CONNECTION_STRING=$1
SYSROOT_HOST_PATH=$2

if [[ -z $SSH_CONNECTION_STRING || -z $SYSROOT_HOST_PATH ]]; then
    echo "Usage:"
    echo "$0 <user@xavier-ip> <sysroot-target-path-host>"
    exit 1
fi

echo "Creating sysroot at $SYSROOT_HOST_PATH from $SSH_CONNECTION_STRING"
echo ""
mkdir -p $SYSROOT_HOST_PATH


#RSYNC_FLAGS=--list-only
RSYNC_FLAGS=

# sync basic sysroot folders
rsync $RSYNC_FLAGS -avz --copy-unsafe-links \
    --include-from $SCRIPT_DIR/sysroot_rsync_include.txt \
    --exclude="*" \
    --prune-empty-dirs \
    -e ssh $SSH_CONNECTION_STRING:/ $SYSROOT_HOST_PATH/

# sync ros2 to sysroot
rsync $RSYNC_FLAGS -avz --copy-unsafe-links \
    -e ssh $SSH_CONNECTION_STRING:/opt/ros2_ws $SYSROOT_HOST_PATH/opt/
