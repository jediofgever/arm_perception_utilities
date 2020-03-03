#!/bin/bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal -x sh -c "roslaunch arm_perception_utilities robot_simulation_pick_place.launch; bash" 
