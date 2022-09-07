#!/bin/sh
echo "Launching my_robot world.launch"
xterm -e "roslaunch home_service_robot world.launch" &
sleep 10

echo "Launching amcl.launch"
xterm -e "roslaunch home_service_robot amcl.launch" &
sleep 5

echo "Launching rviz"
xterm -e "roslaunch home_service_robot rviz.launch" &
sleep 5

echo "Launching add_markers node"
xterm -e "rosrun add_markers add_markers" &
sleep 5

echo "Launching pick_objects node"
xterm -e "rosrun pick_objects pick_objects" &
