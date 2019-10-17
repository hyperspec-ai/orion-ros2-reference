#!/bin/bash
docker build -t orion:infra ./infra_image/
docker build -t projectorion/webservice:develop ./infra_image/
docker push projectorion/webservice:develop
