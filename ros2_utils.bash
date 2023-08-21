#!/usr/bin/env bash

# ROS 2 run

function rrun {
  if [ $# -eq 0 ]; then
    PKG_NAME=$(ros2 pkg list | fzf)
    [[ -z "$PKG_NAME" ]] && return
    history -s "rrun $PKG_NAME"
    rrun $PKG_NAME
  elif [ $# -eq 1 ]; then
    PKG_AND_EXE=$(ros2 pkg executables | grep $1 | fzf)
    [[ -z "$PKG_AND_EXE" ]] && return
    CMD="ros2 run $PKG_AND_EXE"
    echo "$CMD"
    $CMD
    history -s rrun
    history -s $CMD
  fi
}

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
    [[ -z "$TOPIC" ]] && return
    CMD="ros2 topic echo $TOPIC"
    echo $CMD
    $CMD
    history -s rtecho
    history -s $CMD
}

function rtinfo {
    TOPIC=$(ros2 topic list | fzf)
    [[ -z "$TOPIC" ]] && return
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
    [[ -z "$NODE" ]] && return
    CMD="ros2 node info $NODE"
    echo $CMD
    $CMD
    history -s rninfo
    history -s $CMD
}

# Services

function rslist {
    CMD="ros2 service list"
    echo $CMD
    $CMD
    history -s rslist
    history -s $CMD
}

# Parameters

function rplist {
    NODE=$(ros2 node list | fzf)
    [[ -z "$NODE" ]] && return
    CMD="ros2 param list $NODE --param-type"
    echo $CMD
    $CMD
    history -s rplist
    history -s $CMD
}

function rpget {
    NODE=$(ros2 node list | fzf)
    [[ -z "$NODE" ]] && return
    PARAM=$(ros2 param list $NODE | fzf)
    [[ -z "$PARAM" ]] && return
    CMD="ros2 param get $NODE $PARAM"
    echo $CMD
    $CMD
    history -s rpget
    history -s $CMD
}

function rpset {
    NODE=$(ros2 node list | fzf)
    [[ -z "$NODE" ]] && return
    PARAM=$(ros2 param list $NODE | fzf)
    [[ -z "$PARAM" ]] && return
    echo -n "value: "
    read VALUE
    CMD="ros2 param set $NODE $PARAM $VALUE"
    echo $CMD
    $CMD
    history -s rpset
    history -s $CMD
}

function rnkill {
    NODE_TO_KILL_RAW=$(ros2 node list | fzf)
    [[ -z "$NODE_TO_KILL_RAW" ]] && return
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
    CMD="rosdep install --from-paths src --ignore-src -r -y"
    echo $CMD
    $CMD
    history -s rosdep_install
    history -s $CMD
}
