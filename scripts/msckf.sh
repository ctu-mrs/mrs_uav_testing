#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

UAV_NAME=$1
SESSION_NAME=monitor_$UAV_NAME

~/.i3/detacher.sh 1 "~/.scripts/set_ros_master_uri.sh $UAV_NAME"

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME; export ATHAME_ENABLED=0"

# define commands
# 'name' 'command'
input=(
  'RVIZ' "waitForRos; roscd mrs_testing; ${SCRIPT_PATH}/change_uav.sh $UAV_NAME; rosrun rviz rviz -d ${SCRIPT_PATH}/../rviz/msckf.rviz
  "
  'Juggler' "waitForRos; sleep 2;  ${SCRIPT_PATH}/change_uav.sh $UAV_NAME; i3 workspace "9"; rosrun plotjuggler PlotJuggler -l ${SCRIPT_PATH}/../plot_juggler/msckf.xml
  "
  'Reconfigure' " waitForRos; rosrun rqt_reconfigure rqt_reconfigure
  "
  'Layout' "waitForRos; i3 workspace '9'; sleep 10;  ~/.i3/layout_manager.sh ${SCRIPT_PATH}/../layouts/layout-RVIZ_RQT_JUGGLER.json
  "
)

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s "$SESSION_NAME" -d
  echo "Starting new session."
else
  SESSION_NAME=$(tmux display-message -p '#S')
  echo "Already in tmux, attaching to the current session"
fi

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
	((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
	tmux new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

# add pane splitter for mrs_status
tmux new-window -t $SESSION_NAME:$((${#names[*]}+1)) -n "mrs_status"

# clear mrs status file so that no clutter is displayed
truncate -s 0 /tmp/status.txt

# split all panes
pes=""
for ((i=0; i < ((${#names[*]}+2)); i++));
do
  pes=$pes"tmux split-window -d -t $SESSION_NAME:$(($i))"
  pes=$pes"tmux send-keys -t $SESSION_NAME:$(($i)) "${pre_input};"'tail -F /tmp/status.txt'"
  pes=$pes"tmux select-pane -U -t $(($i))"
done

tmux send-keys -t $SESSION_NAME:$((${#names[*]}+1)) "${pre_input};${pes}"

sleep 6

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
	tmux pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
	tmux send-keys -t $SESSION_NAME:$(($i+1)) "${pre_input};${cmds[$i]}"
done

pes="sleep 1;"
for ((i=0; i < ((${#names[*]}+2)); i++));
do
  pes=$pes"tmux select-window -t $SESSION_NAME:$(($i))"
  pes=$pes"tmux resize-pane -U -t $(($i)) 150"
  pes=$pes"tmux resize-pane -D -t $(($i)) 7"
done

pes=$pes"tmux select-window -t $SESSION_NAME:4"
pes=$pes"waitForRos; export UAV_NAME=$UAV_NAME; roslaunch mrs_status f550.launch >> /tmp/status.txt"

tmux send-keys -t $SESSION_NAME:$((${#names[*]}+1)) "${pre_input};${pes}"

tmux -2 attach-session -t $SESSION_NAME

clear
