#!/bin/bash

echo "export ROS_WS=${HOME}/roborts_ws" >> ~/.bashrc

echo "alias cm='cd ${ROS_WS} && catkin_make'" >> ~/.bash_aliases
echo "alias ss='source ${ROS_WS}/devel/setup.bash'" >> ~/.bash_aliases
echo "alias roslaunch='ss && roslaunch'" >> ~/.bash_aliases

