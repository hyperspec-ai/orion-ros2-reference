#!/bin/bash
./deploy_to_docker.sh ./install/rtcm/lib/libRtcmNode.so rtcm_v1.tar.gz repo:/var/www/html
./deploy_to_docker.sh "./install/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_cpp.so ./install/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_fastrtps_cpp.so ./install/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_introspection_cpp.so" rtcm_msgs_v1.tar.gz repo:/var/www/html
