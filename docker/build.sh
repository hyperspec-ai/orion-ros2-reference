echo "ConfigNode Directory mapped to $WS_KROS"
docker run -v $WS_KROS:/opt/ws_kros -v $ORION_SYSROOT:/opt/sysroot --rm orion:dev "bash /opt/ws_kros/docker/run_in_docker_scripts/build.sh"
