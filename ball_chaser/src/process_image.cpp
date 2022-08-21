#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO("Moving robot linear_x: %1.2f, angular_z: %1.2f", lin_x, ang_z);
    
    // Request service
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service
    if (!client.call(srv)){
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // pixel encoding rgb8 -> 3 channels 
    int white_pixel = 255;
    int red_channel = 0;
    int green_channel = 1;
    int blue_channel = 2;
    int found_white_pixel = 0;
    

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    for (unsigned int row=0; row < img.height; row++){
	for (unsigned int col=0; col < img.step; col++){
	    int index = row * img.step + col;
	    if (img.data[index+red_channel] == white_pixel && img.data[index+green_channel] == white_pixel && img.data[index+blue_channel] == white_pixel){
		ROS_INFO("FOUND WHITE PIXEL");
		found_white_pixel = index;
		break;
	    }
	}
    }
    if(found_white_pixel==0){
        ROS_INFO("No white pixel found");
        drive_robot(0.0, 0.0);
	return;
    }
    // determine where the pixel falls
    int pixel = found_white_pixel % img.width;
    if (pixel < (img.step / 3)){
        ROS_INFO("White pixel found on left side");
  	drive_robot(0.5, 0.5);
    } else if (pixel < (img.step / 3)*2) {
    	ROS_INFO("White pixel found in middle panel");
	drive_robot(1.0, 0.0);
    } else {
    	ROS_INFO("White pixel found on right side");
	drive_robot(0.5, -0.5);
    }

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
