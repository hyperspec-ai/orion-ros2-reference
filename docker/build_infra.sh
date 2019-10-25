#!/bin/bash

# if AUTOMATED_BUILD is not set, then we're running a local build
if [ -z "$AUTOMATED_BUILD" ]; then
    AUTOMATED_BUILD=0
fi

# build the infra image
docker build -t orion:infra ./infra_image/
docker build -t projectorion/webservice:develop ./infra_image/

# push image dto dockerhub (only when running on jenkins)
if [ "$AUTOMATED_BUILD" -neq "0" ];
    docker push projectorion/webservice:develop
fi
