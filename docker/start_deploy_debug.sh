echo "ConfigNode Directory mapped to $WS_KROS"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pwd
bash $DIR/build.sh
bash deploy_all.sh
cp config_x86/repository/repository_empty.json config_x86/repository/repository.json
docker run --net orion_nw --ip 172.18.0.10 -it -v $WS_KROS:/opt/ws_kros --privileged -v /dev:/dev --rm orion:dev bash "docker/run_in_docker_scripts/start_debug.sh"