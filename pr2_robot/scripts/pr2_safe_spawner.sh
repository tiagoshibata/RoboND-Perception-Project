#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
roslaunch pr2_robot pick_place_demo.launch & sleep 20 &&
roslaunch pr2_moveit pr2_moveit.launch & sleep 40 &&
rosrun pr2_robot pr2_motion
