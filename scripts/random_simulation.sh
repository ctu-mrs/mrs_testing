#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

PACKAGE_PATH=$( rospack find mrs_uav_testing )

cp $PACKAGE_PATH/rviz/default_random_simulation.rviz /tmp/default_random_simulation.rviz

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/default_random_simulation.rviz
