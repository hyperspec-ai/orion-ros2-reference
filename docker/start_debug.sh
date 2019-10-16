echo "ConfigNode Directory mapped to $WS_KROS"
pwd
bash deploy_all_pc.sh
cp config/repository/repository_empty.json config/repository/repository.json
docker run --net orion_nw --ip 172.18.0.10 -it -v $WS_KROS:/opt/ws_kros --privileged -v /dev:/dev --rm orion:dev gdbserver :1234 "install/config/lib/config/ConfigNode_exec ./config"