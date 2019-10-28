echo "ConfigNode Directory mapped to $WS_KROS"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cp config_arm/repository/repository_empty.json config_arm/repository/repository.json
#sshpass -p $ORION_XAVIER1_PASSWORD rsync --progress -avz  -e ssh $WS_KROS/install_arm/ $ORION_XAVIER1_USER@$ORION_XAVIER1_IP:/opt/orion
sshpass -p $ORION_XAVIER1_PASSWORD rsync --progress -avz  -e ssh $WS_KROS/config_arm/ $ORION_XAVIER1_USER@$ORION_XAVIER1_IP:/opt/orion/config_arm
sshpass -p $ORION_XAVIER1_PASSWORD ssh  -t $ORION_XAVIER1_USER@$ORION_XAVIER1_IP 'cd /opt/orion && export COLCON_CURRENT_PREFIX="/opt/orion" &&
source /opt/orion/local_setup.sh && source /opt/ros2_ws/install/setup.bash && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/orion/lib_message && gdbserver :1234 /opt/orion/config/lib/config/ConfigNode_exec /opt/orion/config_arm'

