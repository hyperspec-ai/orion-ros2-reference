docker run --net orion_nw --ip 172.18.0.10 -it -v $WS_KROS:/opt/ws_kros --privileged -v /dev:/dev  --rm orion:dev 
