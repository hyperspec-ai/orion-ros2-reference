#!/bin/bash
docker build -t orion:dev ./dev_image/
docker build -t projectorion/build_environment:develop ./dev_image/
docker push projectorion/build_environment:develop
