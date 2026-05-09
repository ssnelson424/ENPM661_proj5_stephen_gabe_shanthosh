# ENPM661 Project 05: Map-Adaptive APEI-RRT* Path Planning for Rapid TurtleBot Navigation on a Competition Game Board
Implementation of RRT,RRT*, and APEI-RRT* path planning and simulation for a TurtleBot3 Waffle robot on the competition map.

## Team Members

Stephen Snelson - Directory ID: ssnelson@umd.edu - UID: 12254074
Gabriel Szybalski - Directory ID: gszybals@umd.edu - UID: 117904949
Shanthosh Raaj - Directory ID: sandy1@umd.edu - UID: 122010895

##Github Repository
https://github.com/ssnelson424/ENPM661_proj5_stephen_gabe_shanthosh.git


## Overview

This repository implements multiple algorithms for path planning for a differential-drive TurtleBot3 Waffle robot using the 8-action RPM set from previous projects. The planner validates user inputs, models obstacle space with convex polygon half-plane equations, generates collision-free paths using RRT, RRT*, and APEI-RRT*, and executes the planned path in Gazebo with feedback tracking.

## Dependencies

- Ubuntu 22.04
- Python 3.10+
- ROS 2 Humble
- Gazebo
- `colcon`
- `matplotlib`
- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf2`

Example setup commands:

```bash
source /opt/ros/humble/setup.bash
pip3 install matplotlib
sudo apt install python3-colcon-common-extensions
```

## Run Instructions

### Build the TurtleBot3 package

1. Create a ROS2 workspace:

```bash
mkdir -p ~/project5ws/src
cd ~/project5ws/src
```

2. Clone the repository:

```bash
git clone https://github.com/ssnelson424/ENPM661_proj5_stephen_gabe_shanthosh.git
```

3. Build the package and initialize the gazebo space:

```cd ~/proj5ws

rm -rf build install log

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=~/proj5ws/src/FINAL/Project_5_export/Project_5_export/turtlebot3_proj5_661/models:$GAZEBO_MODEL_PATH

ros2 launch turtlebot3_project3 competition_world.launch.py x_pose:=0.5 y_pose:=0.0 yaw:=0.0

```

### Run Path Planner
Modify run.py in the following desitnation: proj5ws/src/FINAL/Project_5_export/Project_5_export/algorithm_run_1
	There is a block with the below code, comment out 2/3 depending on if running RRT, RRTStar, or RRTSTarAPEI
	rrt = RRT(game_board,turtlebot)
	rrt = RRTStar(game_board,turtlebot)
    	rrt = RRTStarAPEI(game_board,turtlebot)
Run the planner from the package in a separate window from the Gazebo terminal:
```
cd /home/gszybalski/proj5ws/src/FINAL/Project_5_export/Project_5_export/algorithm_run_1

source /opt/ros/humble/setup.bash
source /home/gszybalski/proj5ws/install/setup.bash
```


## Outputs

 - Terminal output showing planning statistics, including path length and calculation time
- Saved command sequence for the gazebo robot
- Saved path plot image
- Gazebo simulation of the TurtleBot3 following the determined path


## Video

-RRT-https://drive.google.com/file/d/1E-Ca0Ke7d8JETdB2L1tQ7VytxtRyd5A6/view?usp=drive_link
-RRT*-https://drive.google.com/file/d/10oAzRkZlW9SvdwsRhWKh9DTnqrdKzdG9/view?usp=drive_link
-APEI-RRT*- https://drive.google.com/file/d/1OlWELLakpAeAozAYtG83GfXeXCi3aI9l/view?usp=drive_link

