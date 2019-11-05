#!/bin/bash

PAHO_C_SRC=/tmp/paho.mqtt.c

git clone https://github.com/eclipse/paho.mqtt.c.git $PAHO_C_SRC
cd $PAHO_C_SRC
cmake -Bbuild -H. -DPAHO_WITH_SSL=ON
sudo cmake --build build/ --target install
sudo ldconfig

PAHO_CPP_SRC=/tmp/paho.mqtt.cpp
git clone https://github.com/eclipse/paho.mqtt.cpp $PAHO_CPP_SRC
cd $PAHO_CPP_SRC
cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=FALSE -DPAHO_BUILD_SAMPLES=TRUE -DCMAKE_PREFIX_PATH=$PAHO_C_SRC/build/install
sudo cmake --build build/ --target install
sudo ldconfig
