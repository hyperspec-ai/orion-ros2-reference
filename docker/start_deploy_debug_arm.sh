echo "ConfigNode Directory mapped to $WS_KROS"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pwd
bash $DIR/build_arm.sh
bash deploy_all.sh
bash $DIR/start_debug_arm.sh
