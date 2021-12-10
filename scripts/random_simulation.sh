#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

if [ -x "$(whereis nvim | awk '{print $2}')" ]; then
 VIM_BIN="$(whereis nvim | awk '{print $2}')"
 HEADLESS="--headless"
elif [ -x "$(whereis vim | awk '{print $2}')" ]; then
 VIM_BIN="$(whereis vim | awk '{print $2}')"
 HEADLESS=""
fi

cp $SCRIPT_PATH/../rviz/default_random_simulation.rviz /tmp/default_random_simulation.rviz

$VIM_BIN -u "$GIT_PATH/linux-setup/submodules/profile_manager/epigen/epigen.vimrc" $HEADLESS -Ens -c "%s/uav[0-9]\+/$UAV_NAME/g" -c "wqa" -- "/tmp/default_random_simulation.rviz"
