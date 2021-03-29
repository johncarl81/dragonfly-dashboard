#!/bin/bash

sudo route add -net 224.0.0.0 netmask 240.0.0.0 wlan1
roscore&
sleep 3
rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.0 &
sleep 3
rosrun master_sync_fkie master_sync &
sleep 3
roslaunch rosbridge_server rosbridge_websocket.launch&
