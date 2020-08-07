#!/bin/bash
source /opt/ros/kinetic/setup.bash
catkin_make
wait $!
echo "source devel" &
source devel/setup.bash
source devel/setup.bash
