#!/bin/bash
set -e

export ROS_MASTER_URI=http://172.18.0.2:11311

exec "$@"
