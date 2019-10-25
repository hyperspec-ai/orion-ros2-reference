#!/bin/bash
#ensure that infra strcuture docker container is running, will fail in case already running, can be ignored
docker run --name repo --net orion_nw --ip 172.18.0.11 -it  --rm -d orion:infra
sleep 5
docker exec repo mkdir -p /var/www/html/arm
docker exec repo mkdir -p /var/www/html/x86
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
bash $DIR/deploy_to_docker.sh $DIR/install_arm/rtcm/lib/libRtcmNode.so rtcm_v1.tar.gz repo:/var/www/html/arm/
bash $DIR/deploy_to_docker.sh $DIR/install_arm/ublox/lib/libUbxGpsNode.so ublox_v1.tar.gz repo:/var/www/html/arm/
bash $DIR/deploy_to_docker.sh "$DIR/install_arm/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_cpp.so $DIR/install_arm/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_fastrtps_cpp.so $DIR/install_arm/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_introspection_cpp.so" rtcm_msgs_v1.tar.gz repo:/var/www/html/arm/
bash $DIR/deploy_to_docker.sh $DIR/install_x86/rtcm/lib/libRtcmNode.so rtcm_v1.tar.gz repo:/var/www/html/x86/
bash $DIR/deploy_to_docker.sh $DIR/install_x86/ublox/lib/libUbxGpsNode.so ublox_v1.tar.gz repo:/var/www/html/x86/
bash $DIR/deploy_to_docker.sh "$DIR/install_x86/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_cpp.so $DIR/install_x86/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_fastrtps_cpp.so $DIR/install_x86/rtcm_msgs/lib/librtcm_msgs__rosidl_typesupport_introspection_cpp.so" rtcm_msgs_v1.tar.gz repo:/var/www/html/x86/
