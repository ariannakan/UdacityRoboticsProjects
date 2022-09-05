#!/bin/sh
echo "Launching my_robot world.launch"
xterm -e "roslaunch home_service_robot world.launch" &
sleep 10

echo "Launching amcl.launch"
#xterm -e "roslaunch my_robot gmapping.launch" &
xterm -e "roslaunch home_service_robot amcl.launch" &
sleep 5

echo "Launching turtlebot view_navigation.launch"
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

echo "Launching pick_objects node"
xterm -e "rosrun pick_objects pick_objects"
