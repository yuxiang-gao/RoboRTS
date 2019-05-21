#!/bin/bash

echo "export ROS_WS=${HOME}/roborts_ws" >> ~/.bashrc
export ROS_WS=${HOME}/roborts_ws
echo "alias cm='cd ${ROS_WS} && catkin_make'" >> ~/.bash_aliases
echo "alias ss='source ${ROS_WS}/devel/setup.bash'" >> ~/.bash_aliases
echo "alias roslaunch='ss && roslaunch'" >> ~/.bash_aliases

bash -c "modprobe uvcvideo nodrop=1 timeout=6000 quirks=0x80"
