
docker run -it -v $WS_KROS:/opt/ws_kros --privileged -v /dev:/dev --rm \
-e DISPLAY  \
-v /tmp/.X11-unix:/tmp/.X11-unix \
ros2:dasdev