#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>


// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;



bool handle_drive(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveBall request received - linear:%1.2f, angular:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Publish velocity 
    std_msgs::Float64 linear_x, angular_z;

    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
  
    motor_command_publisher.publish(motor_command);
   
    // Return a response message
    res.msg_feedback = "Velocity :  " + std::to_string(req.linear_x) + " angular velocity : " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);
    ros::Duration(1).sleep();

    return true;

}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive);
  
    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function

    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    
    
    ros::spin();

    return 0;
}
