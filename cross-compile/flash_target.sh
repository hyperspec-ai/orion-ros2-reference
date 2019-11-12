#!/bin/bash

L4T_BASE_DIR=~/nvidia/nvidia_sdk/JetPack_4.2.2_Linux_GA_P2888/Linux_for_Tegra

SYSROOT_HOST_DIR=$1

if [ -z $SYSROOT_HOST_DIR ]; then
    echo "Usage:"
    echo "$0 <sysroot-host-path>"
    exit 1
fi

echo "Flash Xavier with rootfs '$SYSROOT_HOST_DIR'"
export ROOTFS_DIR=~/code/sysroots/nice-rootfs

pushd $L4T_BASE_DIR
./flash.sh jetson-xavier mmcblk0p1
popd
