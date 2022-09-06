#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker marker;

void goal_state_callback(cons pick_objects::goal_state_msg::ConstPtr& msg){
  if(msg->goal_state == 1){
    if(msg->goal_id == 1){
      ROS_INFO("Picked up package.");
      marker.color.a = 0.0;
    } else if (msg->goal_id == 2){
      ROS_INFO("Dropped off package.");
      marker.pose.position.x = -3.0;
      marker.color.a = 1.0;
    }
  }
  
  

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Define a subscriber to get robot goal state
  ros::Subscriber sub = n.subscribe("/goal_state",1,goal_state_callback);  

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "marker";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 3.5;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (ros::ok())
  {

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    r.sleep();
  }
}
