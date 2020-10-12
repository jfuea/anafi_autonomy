#!/bin/bash
response=$(fdc ping)
echo $response
echo "vari$1"
if [ "$response" = "PONG" ]; then
    sphinx $1 /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::stolen_interface=""
else
# start service
    printf "service not started, need sudo privileges"
    printf "not working, initialize first with \nsudo systemctl start firmwared.service\n"
    # sudo systemctl start firmwared.service
fi