# ros2-aliases
Collection of functions and aliases for ROS2 development

![](https://github.com/tonynajjar/ros2-aliases/blob/main/usage.gif)

# Prerequisites

- [fzf](https://github.com/junegunn/fzf#installation)
For Ubuntu simply: `sudo apt-get install fzf`. For more install options refer to the documentation

- Bash. Zsh is not yet supported

# Installation

- Clone the repo: `git clone https://github.com/tonynajjar/ros2-aliases ~/ros2-aliases`
- Add ros2_utils.bash to your bashrc: `echo 'source ~/ros2-aliases/ros2_utils.bash ' >> ~/.bashrc`

# Usage

## Topics

| Command | Alias |
| --- | --- |
| `ros2 topic list` | `rtlist` |
| `ros2 topic echo` | `rtecho`|
| `ros2 topic info` | `rtinfo`|

## Nodes

| Command | Alias |
| --- | --- |
| `ros2 node list` | `rnlist` |
| `ros2 node info` | `rninfo`|
| Killing a node | `rnkill`|

`rnkill` is a prototype and might not work as intended. It is an attempt to emulate ROS1's `rosnode kill`

## TF

| Command | Alias | Arguments |
| --- | --- | --- |
| `ros2 run tf2_tools view_frames` | `view_frames` | namespace of TF topic [Optional] |
| `ros2 run tf2_ros tf2_echo` | `tf2_echo`| frame 1 [Required], frame 2 [Required], namespace of TF topic [Optional] |

## Colcon

| Command | Alias | Arguments |
| --- | --- | --- |
| `colcon build --symlink-install` | `cb` |
| `colcon build --symlink-install --packages-select` | `cb`| package 1 [Required] ... package n [Optional] |

