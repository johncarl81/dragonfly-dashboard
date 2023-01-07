#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspace/dragonfly/install/setup.bash

exec "$@"
