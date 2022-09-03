#!/bin/sh
cd ../../ # cd to catkin_ws from scripts folder

echo "Launching turtlebot_world.launch"
xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/MyWorld4" &
sleep 5

echo "Launching turtlebot gmapping_demo.launch and setting params"
xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
#xterm -e "rosrun gmapping slam_gmapping" &
sleep 5

echo "Launching turtlebot view_navigation.launch"
xterm -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

echo "Launching turtlebot keyboard_teleop.launch"
xterm -e "source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
