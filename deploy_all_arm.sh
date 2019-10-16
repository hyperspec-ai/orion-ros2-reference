#!/bin/bash
./deploy_to_infra.sh ./install/rtcm/lib/libRtcmNode.so rtcm_v1.tar.gz 
./deploy_to_infra.sh ./install/ublox/lib/libUbxGpsNode.so ublox_v1.tar.gz 
./deploy_to_infra.sh ./install/flir/lib/libFlirNode.so flir_v1.tar.gz 
