#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class RosNodeHandle
{
private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::Subscriber sub;
public:
    RosNodeHandle()
    {
        client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        sub = n.subscribe("/camera/rgb/image_raw", 10, &RosNodeHandle::process_image_callback, this);
        ROS_INFO("Ready to process image");
    }
    void drive_robot(float lin_x, float ang_z)
    {
        ROS_INFO("drive.lin_x: %4.2f, drive.ang_z: %4.2f", lin_x, ang_z);  
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        if(!client.call(srv))
            ROS_ERROR("Failed to call service command_robot");

    }

    void process_image_callback(const sensor_msgs::Image img)
    {
        int white_pixel = 255;
        int num_white_pixel = 0;
        int mean_pos_y = 0;

        // ROS_INFO("img.width: %d, img.height: %d", img.width, img.height);
        // ROS_INFO("img.step: %d", img.step);

        for (int i = 0; i < img.height; i++)
        {
            for (int j = 0; j < img.width; j++)
            {
                if(img.data[i * img.step + 3 * j] == white_pixel
                && img.data[i * img.step + 3 * j + 1] == white_pixel
                && img.data[i * img.step + 3 * j + 2] == white_pixel)
                {
                    num_white_pixel++;
                    mean_pos_y += j;
                    // ROS_INFO("ball.height: %d, ball.width: %d", i, j);
                    // return;
                }
            }
        }

        ROS_INFO("num_white_pixel: %d", num_white_pixel);
        if (num_white_pixel)
        {
            mean_pos_y /= num_white_pixel;
            if (mean_pos_y < (img.width / 4))
            {
                ROS_INFO("Turn Left");
                drive_robot(0.0, 0.2);
            }
            else if (mean_pos_y > (img.width * 3 / 4))
            {
                ROS_INFO("Turn Right");
                drive_robot(0.0, -0.2);
            }
            else if (num_white_pixel < 100000)
            {
                ROS_INFO("Go Forward");
                drive_robot(0.1, 0.0);
            }
            else
            {
                ROS_INFO("Stop");
                drive_robot(0.0, 0.0);
            }
            
        }
        else
        {
            ROS_INFO("No Ball!!");
            drive_robot(0.0, 0.0);
        }
        // ros::Duration(5).sleep();
        return;
        
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");

    RosNodeHandle rosNodeHandle;

    ros::spin();
}