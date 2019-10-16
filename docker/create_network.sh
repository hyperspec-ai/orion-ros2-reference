#!/bin/bash
docker network create \
  --subnet=172.18.0.0/24 \
  --opt com.docker.network.bridge.name="orion_br"\
  orion_nw
  sudo brctl addif orion_br eth_orion

