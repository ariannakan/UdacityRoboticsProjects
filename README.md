# Home Service Robot

## Project Goals
The goal of this project is to program a robot to map an environment and navigate to pick up and drop off virtual objects. 

This project is composed of multiple steps: 
1. Design a simulation world with the Building Editor in Gazebo.
2. Teleoperate your robot and manually test SLAM - creating a functional 2D map of the environment.
3. Use the ROS Navigation stack (AMCL) to test your robot's ability to reach and localize itself with respect to 2 different goals.
4. Write a c++ node (pick_objects.cpp) to communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach.
5. Write a c++ node that communicates with the pick_objects node to publish markers in RViz - simulating package pickup and dropoff.

## Dependencies
Uses ROS Kinetic on Ubuntu 16.04 (Xenial)

### Packages
#### Official ROS packages

Import these packages and install them in the src directory of your catkin workspace. Be sure to clone the full GitHub directory and not just the package itself.

- [gmapping](http://wiki.ros.org/gmapping): Easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
- [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard): Manually control a robot using keyboard commands.
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers): Load a preconfigured rviz workspace. It will automatically load the robot model, trajectories, and map.
- [amcl](http://wiki.ros.org/amcl): Takes in laser scans and uses a particle filter to track the pose of the robot against the map.

#### Your Packages and Directories
- pick_objects: Node that will communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach.
- add_markers: Node that will publish markers in RViz to simulate package pickup and dropoff - based on the state of the robot with respect to its goals.
- home_service_robot: Houses the launch files, map files, robot files, and scripts.
  - map: Store your gazebo world file and the map generated from SLAM.
  - scripts: Store your shell scripts.
  - rvizConfig: Store your customized rviz configuration files.

### Build
1. Create and initialize catkin_ws 
    ```
    $ mkdir -p /home/workspace/catkin_ws/src
    $ cd /home/workspace/catkin_ws/src
    $ catkin_init_workspace
    ```
2. Clone this repo into the src directory
    ```
    $ git clone https://github.com/ariannakan/UdacityRoboticsProjects.git
    ```
3. Clone the remaining ROS packages into the src directory
4. If you are working with a native ROS installation or using a VM, some of the following package might need to be installed.
    ```
    $ sudo apt-getinstall ros-kinetic-navigation
    $ sudo apt-getinstall ros-kinetic-map-server
    $ sudo apt-getinstall ros-kinetic-move-base
    $ sudo apt-getinstall ros-kinetic-amcl
    ```
5. Run catkin_make
    ```
    $ cd /home/workspace/catkin_ws
    $ catkin_make
    $ source devel/setup.bash
    ```
6. Run the scripts:
    - **launch.sh** - Launches gazebo and rviz in separate terminals.
    - **test_slam.sh** - Launches your world and your robot in Gazebo, slam_gmapping node to perform slam and create a 2D map of the environment, rviz to visualize the robot while building the map, and teleop node to control the robot.
        Run `rosrun map_server map_saver -f <map_name>` to save map.
    - **test_navigation.sh** - Launches your world in Gazebo, amcl node to perform localization, rviz to visualize navigating with the ros navistack.
    - **pick_objects.sh** - Launches your world in Gazebo, amcl node to perform localization, rviz to visualize navigating, and pick_objects node to autonomously send successive goal commands to the robot.
    - **add_markers.sh** - Launches your world in Gazebo, amcl node to perform localization, rviz to visualize the markers, and add_markers node to draw virtual boxes in the pickup and dropoff locations.
    - **home_service.sh** - Launches your world in Gazebo, amcl node to perform localization, rviz to visualize the markers and the robot navigating, pick_objects node to autonomously send pickup and dropoff goals to the robot, and add_markers node to draw virtual boxes in the pickup and dropoff locations.




