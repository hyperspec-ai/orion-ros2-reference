#!/bin/

export COLCON_CURRENT_PREFIX="/opt/ws_kros/install_x86"
source /opt/ws_kros/install_x86/local_setup.sh
gdbserver :1234 /opt/ws_kros/install_x86/config/lib/config/ConfigNode_exec ./config_x86