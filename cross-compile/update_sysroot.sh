#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

L4T_BASE_DIR=~/nvidia/nvidia_sdk/JetPack_4.2.2_Linux_GA_P2888/Linux_for_Tegra

# set this to false if you want to sync target to sysroot as well
SKIP_SYNC_FROM_TARGET="false"

# read arguments
SSH_CONNECTION=$1
SSH_PUB_KEY=~/.ssh/id_rsa
SYSROOT_HOST_DIR=$2

if [[ -z $SSH_CONNECTION || -z $SYSROOT_HOST_DIR ]]; then
    echo "Usage:"
    echo "$0 <user@xavier-ip> <sysroot-target-path>"
    exit 1
fi

# check if jetpack is installed and we have a base sysroot available, exit if not
if [ ! -d "$L4T_BASE_DIR" ]; then
    echo "Unable to find Linux for Tegra. Please install Jetpack."
    echo "  - search path was '$L4T_BASE_DIR'"
    exit -1
fi


ROOTFS_SOURCE=$L4T_BASE_DIR/rootfs/
ROOTFS_TARGET=$SYSROOT_HOST_DIR

ADDITIONAL_RSYNC_OPTS=

echo "Sync rootfs base from '$ROOTFS_SOURCE' to '$ROOTFS_TARGET'";
sudo rsync -axvu $ADDITIONAL_RSYNC_OPTS $ROOTFS_SOURCE $ROOTFS_TARGET

if [ "$SKIP_SYNC_FROM_TARGET" == "false"]; then
    echo "Sync rootfs overlay from '$SSH_CONNECTION'"
    sudo rsync -axvuz $ADDITIONAL_RSYNC_OPTS -e "ssh -i $SSH_PUB_KEY" \
        --exclude='/dev' --exclude='/sys' --exclude='/proc' --exclude='/tmp' \
        --exclude='/var/tmp' --exclude='/var/log' --exclude='/var/lib/apt' \
        --exclude='/var/spool' --exclude='/var/cache' \
        --exclude='/var/backups' \
        --rsync-path='sudo rsync' \
        $SSH_CONNECTION:/ $ROOTFS_TARGET
fi
