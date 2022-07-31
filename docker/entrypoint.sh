#!/bin/bash
set -e

export CYCLONEDDS_URI=file://workspace/cyclonedds.xml

source /opt/ros/galactic/setup.bash
source /workspace/dragonfly/install/setup.bash

ros2 daemon start

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

exec "$@"
