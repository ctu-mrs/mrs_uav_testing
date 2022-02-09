#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

cp $SCRIPT_PATH/../rviz/default_random_simulation.rviz /tmp/default_random_simulation.rviz

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/default_random_simulation.rviz
