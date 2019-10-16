#!/bin/bash
echo "Packaging " $1 " to " $2
tar --transform 's/.*\///g' -zcvf /tmp/$2 $1
echo "Copying " $2 " to " $3
docker cp /tmp/$2 $3/$2