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

    int white_pixel = 255;
    vector<int> white_rows;
    vector<int> white_cols;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int row = 0; row <= img.height; ++row) {
        for (int col = 0; col <= img.width; ++col) {
            if (img.data[row][col] == white_pixel) {
                if (!white_rows.empty())
                    if (white_rows.back().c != row){
                        white_rows.push_back(row);
                    }
                if (!white_cols.empty())
                    if (white_cols.back().c != col){
                        white_cols.push_back(col);
                    }
            }
        }
    }

    if (white_rows.empty()){
        ROS_INFO("No white ball in frame - stopping robot");
        drive_robot(0.0, 0.0);
    } else {
        // Calculate midpoint of white ball
        int mid_row = white_rows.begin() + white_rows.size() / 2;
        int mid_col = white_cols.begin() + white_cols.size() / 2;
        ROS_INFO("White ball midpoint at %d,%d", mid_row, mid_col);
        ROS_INFO("White ball edges: row %d - %d, col %d - %d", white_rows.front(), white_rows.back(), white_cols.front(), white_cols.back());

        // Check where midpoint of ball is placed and set robot drive
        if (mid_row < (img.width / 3)){
            drive_robot(0.0, 0.5);
        } else if (mid_row < (img.width / 3)*2) {
            drive_robot(0.5, 0.0);
        } else {
            drive_robot(-0.0, -0.5);
        }
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