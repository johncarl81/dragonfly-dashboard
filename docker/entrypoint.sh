#!/bin/bash
set -e

source /opt/ros/galactic/setup.bash
source /workspace/dragonfly/install/setup.bash

exec "$@"
