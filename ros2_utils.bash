#!/usr/bin/env bash

# Topics

function rtlist {
    CMD="ros2 topic list"
    echo $CMD
    $CMD
    history -s rtlist
    history -s $CMD
}

function rtecho {
    TOPIC=$(ros2 topic list | fzf)
    CMD="ros2 topic echo $TOPIC"
    echo $CMD
    $CMD
    history -s rtecho
    history -s $CMD
}

function rtinfo {
    TOPIC=$(ros2 topic list | fzf)
    CMD="ros2 topic info -v $TOPIC"
    echo $CMD
    $CMD
    history -s rtinfo
    history -s $CMD
}

# Nodes

function rnlist {
    CMD="ros2 node list"
    echo $CMD
    $CMD
    history -s rnlist
    history -s $CMD
}

function rninfo {
    NODE=$(ros2 node list | fzf)
    CMD="ros2 node info $NODE"
    echo $CMD
    $CMD
    history -s rninfo
    history -s $CMD
}


function rnkill {
    NODE_TO_KILL_RAW=$(ros2 node list | fzf)
    NODE_TO_KILL=(${NODE_TO_KILL_RAW//// })
    NODE_TO_KILL=${NODE_TO_KILL[-1]} # extract last word from node name
    NODE_TO_KILL=[${NODE_TO_KILL:0:1}]${NODE_TO_KILL:1}
    # The method used is to parse the PID and use kill <PID>.
    # If more than 1 PID is found, we abort to avoid killing other processes.
    # The parsing checks for any process with the string [/]$NODE_TO_KILL.
    # This can probably be optimized to always find the one node we are looking for.
    PROC_NB=$(ps aux | grep [/]$NODE_TO_KILL | wc -l)
    if [ $PROC_NB -gt 1 ]; then
        echo "This node name matched with more than 1 process. Not killing"
        return
    elif [ $PROC_NB -eq 0 ]; then
        echo "No processes found matching this node name"
        return
    fi
    PROC_PID=$(ps aux | grep [/]$NODE_TO_KILL | awk '{print $2}')
    CMD="kill $PROC_PID"
    echo "Killing $NODE_TO_KILL_RAW with PID $PROC_PID"
    $CMD
    history -s rnlist
    history -s $CMD
}

# TF

function view_frames {
    if [ $# -eq 0 ]; then
        REMAP=""
    else
        REMAP="--ros-args -r /tf:=/$1/tf -r /tf_static:=/$1/tf_static"
    fi
    CMD="ros2 run tf2_tools view_frames $REMAP"
    echo $CMD
    $CMD
    history -s view_frames $@
    history -s $CMD
}

function tf_echo {
    if [ $# -eq 3 ]; then
        REMAP="--ros-args -r /tf:=/$3/tf -r /tf_static:=/$3/tf_static"
    else
        REMAP=""
    fi
    CMD="ros2 run tf2_ros tf2_echo $1 $2 $REMAP"
    echo $CMD
    $CMD
    history -s tf_echo $@
    history -s $CMD
}

# Colcon

function cb {
    if [ $# -eq 0 ]; then
        CMD="colcon build --symlink-install"
    else
        CMD="colcon build --symlink-install --packages-select $@"
    fi
    echo $CMD
    $CMD
    history -s cb $@
    history -s $CMD
}

# Rosdep

function rosdep_install {
    CMD="rosdep install --from-paths src --ignore-src -r"
    echo $CMD
    $CMD
    history -s rosdep_install
    history -s $CMD
}
