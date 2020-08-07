#!/bin/bash

xterm -hold -e "catkin_make" & "roslaunch capbot_gazebo rosbot.launch"
sleep 15
xterm -hold -e "rosrun capbot_setup_tf tf_broadcaster" &
xterm -hold -e "rosbag record -O laserdata /LaserScan /tf" &
xterm -hold -e "rosrun gmapping slam_gmapping scan:=LaserScan" &
xterm -hold -e "rosrun rviz rviz" &

exit 0
