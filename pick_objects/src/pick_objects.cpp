#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

using namespace std;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Function Declaration
int send_to_goal(double x, double y, MoveBaseClient* mbc);

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  };

  // Define a position and orientation for the robot to reach
  double pickup_x = 3.5;
  double pickup_w = 3.5;

  ROS_INFO("Sending pickup goal");
  send_to_goal(pickup_x, pickup_w, &ac);

  ROS_INFO("Picking up package...");
  ros::Duration(5.0).sleep();
  ROS_INFO("Picked up package!");

  double dropoff_x = -3.0;
  double dropoff_w = 1.0;

  ROS_INFO("Sending dropoff goal");
  send_to_goal(dropoff_x, dropoff_w, &ac);

  return 0;
}

int send_to_goal(double x, double y, MoveBaseClient* mbc){
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.orientation.w = y;

   // Send the goal position and orientation for the robot to reach
  mbc->sendGoal(goal);

  // Wait an infinite time for the results
  mbc->waitForResult();

  // Check if the robot reached its goal
  string goal_type = (x == 3.5) ? "pickup" : "dropoff";
  if(mbc->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray - Reached %s goal!", goal_type.c_str());
  else
    ROS_INFO("Darn - Failed to reach %s goal.", goal_type.c_str());
  
  return 0;
}
