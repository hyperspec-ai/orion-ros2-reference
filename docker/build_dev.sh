#!/bin/bash

# if AUTOMATED_BUILD is not set, then we're running a local build
if [ -z "$AUTOMATED_BUILD" ]; then
    AUTOMATED_BUILD=0
fi

# build the docker dev image
docker build -t orion:dev ./dev_image/
docker build -t projectorion/build_environment:develop ./dev_image/

# push image to dockerhub (only when running on jenkins)
if [ "$AUTOMATED_BUILD" -neq "0" ];
    docker push projectorion/build_environment:develop
fi
