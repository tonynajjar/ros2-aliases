#!/usr/bin/env zsh

# ROS 2 run

function rrun {
  if [ $# -eq 0 ]; then
    PKG_NAME=$(ros2 pkg list | fzf)
    [[ -z "$PKG_NAME" ]] && return
    print -s rrun $PKG_NAME
    rrun $PKG_NAME
  elif [ $# -eq 1 ]; then
    PKG_AND_EXE=$(ros2 pkg executables | grep $1 | fzf)
    [[ -z "$PKG_AND_EXE" ]] && return
    CMD=(ros2 run $PKG_AND_EXE)
    echo $CMD
    eval $CMD
    print -s rrun
    print -s $CMD
  fi
}

# Topics

function rtlist {
    CMD=(ros2 topic list)
    echo $CMD
    $CMD
    print -s rtlist
    print -s $CMD
}

function rtecho {
    TOPIC=$(ros2 topic list | fzf)
    CMD=(ros2 topic echo $TOPIC)
    echo $CMD
    $CMD
    print -s rtecho
    print -s $CMD
}

function rtinfo {
    TOPIC=$(ros2 topic list | fzf)
    CMD=(ros2 topic info -v $TOPIC)
    echo $CMD
    $CMD
    print -s rtinfo
    print -s $CMD
}

# Nodes

function rnlist {
    CMD=(ros2 node list)
    echo $CMD
    $CMD
    print -s rnlist
    print -s $CMD
}

function rninfo {
    NODE=$(ros2 node list | fzf)
    CMD=(ros2 node info $NODE)
    echo $CMD
    $CMD
    print -s rninfo
    print -s $CMD
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
    CMD="ros2 param list $NODE --param-type"
    echo $CMD
    $CMD
    history -s rplist
    history -s $CMD
}

function rpget {
    NODE=$(ros2 node list | fzf)
    PARAM=$(ros2 param list $NODE | fzf)
    CMD="ros2 param get $NODE $PARAM"
    echo $CMD
    $CMD
    history -s rpget
    history -s $CMD
}

function rpset {
    NODE=$(ros2 node list | fzf)
    PARAM=$(ros2 param list $NODE | fzf)
    echo -n "value: "
    read VALUE
    CMD="ros2 param set $NODE $PARAM $VALUE"
    echo $CMD
    $CMD
    history -s rpset
    history -s $CMD
}

# TODO: Not working
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
    CMD=(kill $PROC_PID)
    echo "Killing $NODE_TO_KILL_RAW with PID $PROC_PID"
    $CMD
    print -s rnlist
    print -s $CMD
}

# TF

function view_frames {
    if [ $# -eq 0 ]; then
        echo "ros2 run tf2_tools view_frames"
        ros2 run tf2_tools view_frames
        print -s ros2 run tf2_tools view_frames
    else
        echo "ros2 run tf2_tools view_frames --ros-args -r /tf:=/$1/tf -r /tf_static:=/$1/tf_static"
        ros2 run tf2_tools view_frames --ros-args -r /tf:=/$1/tf -r /tf_static:=/$1/tf_static
        print -s run tf2_tools view_frames --ros-args -r /tf:=/$1/tf -r /tf_static:=/$1/tf_static
    fi
    print -s view_frames $@
}

function tf_echo {
    if [ $# -eq 3 ]; then
        echo "ros2 run tf2_ros tf2_echo $1 $2 --ros-args -r /tf:=/$3/tf -r /tf_static:=/$3/tf_static"
        ros2 run tf2_ros tf2_echo $1 $2 --ros-args -r /tf:=/$3/tf -r /tf_static:=/$3/tf_static
    else
        echo "ros2 run tf2_ros tf2_echo $1 $2"
        ros2 run tf2_ros tf2_echo $1 $2
    fi
    print -s tf_echo $@
}

# Colcon

function cb {
    if [ $# -eq 0 ]; then
        colcon build --symlink-install
    else
        colcon build --symlink-install --packages-select $@
    fi
    print -s cb $@
}

# Rosdep

function rosdep_install {
    echo "rosdep install --from-paths src --ignore-src -r -y"
    rosdep install --from-paths src --ignore-src -r -y
    print -s rosdep_install
}
