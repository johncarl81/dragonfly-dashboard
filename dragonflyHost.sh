#!/bin/bash

sudo route add -net 224.0.0.0 netmask 240.0.0.0 wlan1
roscore&
sleep 10
rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.0 &
sleep 10
rosrun master_sync_fkie master_sync &
