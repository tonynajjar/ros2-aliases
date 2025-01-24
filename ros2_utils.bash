#!/usr/bin/env bash

# ROS 2 run

function rrun {
  if [ $# -eq 0 ]; then
    local PKG_NAME=$(ros2 pkg list | fzf)
    [[ -z "$PKG_NAME" ]] && return
    history -s "rrun $PKG_NAME"
    rrun $PKG_NAME
  elif [ $# -eq 1 ]; then
    local PKG_AND_EXE=$(ros2 pkg executables | grep $1 | fzf)
    [[ -z "$PKG_AND_EXE" ]] && return
    local CMD="ros2 run $PKG_AND_EXE"
    echo "$CMD"
    $CMD
    history -s rrun
    history -s $CMD
  fi
}

# Topics

function rtlist {
    local CMD="ros2 topic list"
    echo $CMD
    $CMD
    history -s rtlist
    history -s $CMD
}

function rtecho {
    local TOPIC=$(ros2 topic list | fzf)
    [[ -z "$TOPIC" ]] && return
    CMD="ros2 topic echo $TOPIC"
    echo $CMD
    $CMD
    history -s rtecho
    history -s $CMD
}

function rthz {
    local TOPIC=$(ros2 topic list | fzf)
    [[ -z "$TOPIC" ]] && return
    CMD="ros2 topic hz $TOPIC"
    echo $CMD
    $CMD
    history -s rthz
    history -s $CMD
}

function rtinfo {
    local TOPIC=$(ros2 topic list | fzf)
    [[ -z "$TOPIC" ]] && return
    local CMD="ros2 topic info -v $TOPIC"
    echo $CMD
    $CMD
    history -s rtinfo
    history -s $CMD
}

function rtbw {
    local TOPIC=$(ros2 topic list | fzf)
    [[ -z "$TOPIC" ]] && return
    local CMD="ros2 topic bw $TOPIC"
    echo $CMD
    $CMD
    history -s rtbw
    history -s $CMD
}

# Nodes

function rnlist {
    local CMD="ros2 node list"
    echo $CMD
    $CMD
    history -s rnlist
    history -s $CMD
}

function rninfo {
    local NODE=$(ros2 node list | fzf)
    [[ -z "$NODE" ]] && return
    local CMD="ros2 node info $NODE"
    echo $CMD
    $CMD
    history -s rninfo
    history -s $CMD
}

function rnkill {
    local NODE_TO_KILL_RAW=$(ros2 node list | fzf)
    [[ -z "$NODE_TO_KILL_RAW" ]] && return
    local NODE_TO_KILL=(${NODE_TO_KILL_RAW//// })
    NODE_TO_KILL=${NODE_TO_KILL[-1]} # extract last word from node name
    NODE_TO_KILL=[${NODE_TO_KILL:0:1}]${NODE_TO_KILL:1}
    # The method used is to parse the PID and use kill <PID>.
    # If more than 1 PID is found, we abort to avoid killing other processes.
    # The parsing checks for any process with the string [/]$NODE_TO_KILL.
    # This can probably be optimized to always find the one node we are looking for.
    local PROC_NB=$(ps aux | grep [/]$NODE_TO_KILL | wc -l)
    if [ $PROC_NB -gt 1 ]; then
        echo "This node name matched with more than 1 process. Not killing"
        return
    elif [ $PROC_NB -eq 0 ]; then
        echo "No processes found matching this node name"
        return
    fi
    local PROC_PID=$(ps aux | grep [/]$NODE_TO_KILL | awk '{print $2}')
    local CMD="kill $PROC_PID"
    echo "Killing $NODE_TO_KILL_RAW with PID $PROC_PID"
    $CMD
    history -s rnlist
    history -s $CMD
}

# Services

function rslist {
    local CMD="ros2 service list"
    echo $CMD
    $CMD
    history -s rslist
    history -s $CMD
}

# Parameters

function rplist {
    local NODE=$(ros2 node list | fzf)
    [[ -z "$NODE" ]] && return
    local CMD="ros2 param list $NODE --param-type"
    echo $CMD
    $CMD
    history -s rplist
    history -s $CMD
}

function rpget {
    local NODE=$(ros2 node list | fzf)
    [[ -z "$NODE" ]] && return
    local PARAM=$(ros2 param list $NODE | fzf)
    [[ -z "$PARAM" ]] && return
    local CMD="ros2 param get $NODE $PARAM"
    echo $CMD
    $CMD
    history -s rpget
    history -s $CMD
}

function rpset {
    local NODE=$(ros2 node list | fzf)
    [[ -z "$NODE" ]] && return
    local PARAM=$(ros2 param list $NODE | fzf)
    [[ -z "$PARAM" ]] && return
    echo -n "value: "
    read VALUE
    local CMD="ros2 param set $NODE $PARAM $VALUE"
    echo $CMD
    $CMD
    history -s rpset
    history -s $CMD
}

# Interface

function rishow {
  local INTERFACE=$(ros2 interface list | fzf)
  [[ -z "$INTERFACE" ]] && return
  local CMD="ros2 interface show $INTERFACE"
  echo $CMD
  $CMD
  history -s rishow
  history -s $CMD
}

# TF

function view_frames {
    if [ $# -eq 0 ]; then
        local REMAP=""
    else
        local REMAP="--ros-args -r /tf:=/$1/tf -r /tf_static:=/$1/tf_static"
    fi
    local CMD="ros2 run tf2_tools view_frames $REMAP"
    echo $CMD
    $CMD
    history -s view_frames $@
    history -s $CMD
}

function tf_echo {
    if [ $# -eq 3 ]; then
        local REMAP="--ros-args -r /tf:=/$3/tf -r /tf_static:=/$3/tf_static"
    else
        local REMAP=""
    fi
    local CMD="ros2 run tf2_ros tf2_echo $1 $2 $REMAP"
    echo $CMD
    $CMD
    history -s tf_echo $@
    history -s $CMD
}

# Colcon

function cb {
    CMD="colcon build --symlink-install"
    echo $CMD
    $CMD
    history -s cb $@
    history -s $CMD
}

function cbp {
    if [ $# -eq 0 ]; then
        PACKAGE=$(colcon list -n | fzf)
        [[ -z "$PACKAGE" ]] && return
        local CMD="colcon build --symlink-install --packages-select $PACKAGE"
    else
        local CMD="colcon build --symlink-install --packages-select $@"
    fi
    echo $CMD
    $CMD
    history -s cbp $@
    history -s $CMD
}

function cbs {
    if [ $# -eq 0 ]; then
        PACKAGE=$(colcon list -n | fzf)
        [[ -z "$PACKAGE" ]] && return
        local CMD="colcon build --symlink-install --packages-skip $PACKAGE"
    else
        local CMD="colcon build --symlink-install --packages-skip $@"
    fi
    echo $CMD
    $CMD
    history -s cbs $@
    history -s $CMD
}

function ct {
    CMD="colcon test"
    echo $CMD
    $CMD
    history -s ct $@
    history -s $CMD
}

function ctp {
    if [ $# -eq 0 ]; then
        PACKAGE=$(colcon list -n | fzf)
        [[ -z "$PACKAGE" ]] && return
        local CMD="colcon test --packages-select $PACKAGE"
    else
        local CMD="colcon test --packages-select $@"
    fi
    echo $CMD
    $CMD
    history -s ctp $@
    history -s $CMD
}

function cts {
    if [ $# -eq 0 ]; then
        PACKAGE=$(colcon list -n | fzf)
        [[ -z "$PACKAGE" ]] && return
        local CMD="colcon test --packages-skip $PACKAGE"
    else
        local CMD="colcon test --packages-skip $@"
    fi
    echo $CMD
    $CMD
    history -s cts $@
    history -s $CMD
}

function cl {
    CMD="colcon list -n"
    echo $CMD
    $CMD
    history -s cl $@
    history -s $CMD
}

# Rosdep

function rosdep_install {
    local CMD="rosdep install --from-paths src --ignore-src -r -y"
    echo $CMD
    $CMD
    history -s rosdep_install
    history -s $CMD
}
