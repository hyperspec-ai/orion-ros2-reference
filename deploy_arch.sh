#!/bin/bash -xv

# This is a helper script that collects the nodes from it's install directory,
# compresses them and stores them in the install directory for the specific
# architecture
ARCH=x64
ARCH_INSTALL_PATH=./install/$ARCH

# usage deploy_to_arch <path-to-lib> <tar-filename>
function deploy_path_to_arch() {
    tar --transform 's/.*\///g' -zcvf /tmp/$2 $1
    cp /tmp/$2 $ARCH_INSTALL_PATH/$2
    echo "Deploy succeeded: " $2
}

# usage deploy_to_arch <package-name> <node-name>
function deploy_node_for_arch() {
    deploy_path_to_arch ./install/$1/lib/lib$2.so $2.tar.gz
}

mkdir -p $ARCH_INSTALL_PATH

deploy_node_for_arch config ConfigNode
deploy_node_for_arch rtcm RtcmNode
deploy_node_for_arch ublox UbxGpsNode
deploy_path_to_arch "./install/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_cpp.so ./install/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_fastrtps_cpp.so ./install/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_introspection_cpp.so" rtcm_msgs.tar.gz
