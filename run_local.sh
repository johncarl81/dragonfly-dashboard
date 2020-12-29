#!/bin/bash

roslaunch rosbridge_server rosbridge_websocket.launch &
P0=$1

./gradlew run &
P1=$1
wait $P0 $P1
