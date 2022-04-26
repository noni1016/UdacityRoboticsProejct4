#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"


class RosNodeHandle
{
private:
    ros::NodeHandle n;
    ros::Publisher motor_command_publisher;
    ros::ServiceServer service;
public:
    RosNodeHandle()
    {
        service = n.advertiseService("/ball_chaser/command_robot", &RosNodeHandle::HandleCmdRobotReq, this);
        motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ROS_INFO("Ready to move robot");
    }
    void PublishMotorCommand(geometry_msgs::Twist &motor_command)
    {
        motor_command_publisher.publish(motor_command);
    }
    bool HandleCmdRobotReq(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
    {
        ROS_INFO("DriveToTarget Request received - linear_x : %4.2f, angular_z : %4.2f", req.linear_x, req.angular_z);

        geometry_msgs::Twist motor_command;

        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;
        PublishMotorCommand(motor_command);
        res.msg_feedback = "Velocity linear_x: " + std::to_string(motor_command.linear.x) + ", angular_z: " + std::to_string(motor_command.angular.z);
        ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_bot");

    RosNodeHandle rosNodeHandle;


    ros::spin();
}