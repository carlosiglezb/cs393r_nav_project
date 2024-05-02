#!/bin/bash

tmux -V > /dev/null 2>&1
if [ $? -eq 127 ]
then
    echo "tmux is not installed! Cannot continue"
    exit 127
fi

session="ROS"

tmux has-session -t $session > /dev/null 2>&1
RUNNING=$?

ARG=$1
if [[ "${ARG}" == "stop" ]]
then
    if [ $RUNNING -ne 0 ]
    then
        echo "tmux session is not running."
        exit 0
    fi

    echo "Killing running tmux session"
    tmux kill-session -t $session > /dev/null 2>&1
    exit 0
fi

if [ $RUNNING -eq 0 ]
then
    echo "ERROR: tmux session is already running. Kill current session with ./tmux_session.sh stop"
    exit 127
fi

tmux new-session -d -s $session
tmux set mouse on
window=0

tmux rename-window -t $session:$window 'roscore'
tmux send-keys -t $session:$window '' C-m
#tmux send-keys -t $session:$window 'colcon build && source install/setup.bash && run_cdl_sim' C-m

window=1
tmux new-window -t $session:$window -n 'recharge_node'
tmux send-keys -t $session:$window "source install/setup.bash && source amrl_msgs/install/setup.bash && source ut_turtlebots/install/setup.bash && ros2 run turtlebot4_nav turtlebot4_recharge_monitor_node" C-m

window=4
tmux new-window -t $session:$window -n 'battery_node'
tmux send-keys -t $session:$window "source install/setup.bash && ros2 run turtlebot4_nav battery_discharge_node" C-m

window=2
tmux new-window -t $session:$window -n 'plotjuggler'
tmux send-keys -t $session:$window "ros2 run plotjuggler plotjuggler" C-m

window=3
tmux new-window -t $session:$window -n 'irobot_ut_iface'
tmux send-keys -t $session:$windoe "python ut_turtlebots/ut_turtlebot_api_translator/ut_turtlebot_api_translator/turtlebot_amrl_dock_translator.py" C-m

# give the host user sudo permissions
echo "$uname ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$uname && \
    chmod 0440 /etc/sudoers.d/$uname

if ! [ -z ${TURTLEBOT4_NAV_DOCKER_CONTEXT+x} ]
then
    /bin/bash
fi
