#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

using namespace std;
enum human_state {approaching, talking, goingaway};

// ROS Publisher
ros::Publisher pub;
// Laser information
int laser_range_size;
float laser_range_max;
vector<float> laser_ranges(28);

// A function to retrieve laser information
void laser(const sensor_msgs::LaserScan::ConstPtr msg)
{
    laser_range_size = msg->ranges.size();
    laser_range_max = msg->range_max;
    laser_ranges = msg->ranges;
}

int main(int argc, char **argv)
{
    // ros initialization
    ros::init(argc,argv,"HumanAI");
    ros::NodeHandle n;

    // publishing topic for the second guy movement
    pub = n.advertise<geometry_msgs::Twist>("marco/mmotion",1000);
    // subscribing to laser information
    ros::Subscriber sublaser = n.subscribe("marco/laser",1000,laser);

    // Data for motion
    geometry_msgs::Twist twist;
    // Initial Marco's state
    human_state mstate = approaching, prev_mstate = approaching;
    // talking status
    bool talking_flag = false;

    while(ros::ok())
    {
        int counter = 0;
        // Detecting whether there is an object within the area of the laser
        for(int i=0;i < laser_range_size; i++)
            if(laser_ranges[i] <= laser_range_max/1.8 && !talking_flag)
            {
                mstate = talking;
                break;
            }
            else if(laser_ranges[i] > laser_range_max/1.8)
                counter++;

        if(counter == laser_range_size) talking_flag = false;

        // Marco's movement based on his current and previous state
        switch(mstate)
        {
            case approaching:
                prev_mstate = mstate;
                twist.linear.x = 0.5;
                ROS_INFO("Marco is approaching");
                pub.publish(twist);
                ros::spinOnce();
                break;
            case talking:
                twist.linear.x = 0.0;
                if(prev_mstate == approaching)
                    mstate = goingaway;
                else
                    mstate = approaching;
                talking_flag = true;
                ROS_INFO("Marco is talking");
                pub.publish(twist);
                ros::spinOnce();
                ros::Duration(5).sleep();
                break;
            default:
                prev_mstate = mstate;
                twist.linear.x = -0.5;
                ROS_INFO("Marco is going away");
                pub.publish(twist);
                ros::spinOnce();
                break;

        }
    }

    return 0;
}
