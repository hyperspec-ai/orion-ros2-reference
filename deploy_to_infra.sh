#!/bin/bash
echo "Packaging " $1 " to " $2
tar --transform 's/.*\///g' -zcvf /tmp/$2 $1
sshpass -p "password" ssh master@172.18.0.11 "mkdir -p /var/www/html/arm"
sshpass -p "password" ssh master@172.18.0.11 "mkdir -p /var/www/html/x86"
sshpass -p "password" scp /tmp/$2 master@172.18.0.11:/var/www/html/arm
