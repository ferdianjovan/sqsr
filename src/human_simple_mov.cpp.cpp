#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

using namespace std;
enum human_state {approaching, talking, goingaway, ready};

// ROS Publisher
ros::Publisher pub;
// Initial Marco's state
human_state mstate = approaching;

// Movement function based on laser information
void laser(const sensor_msgs::LaserScan::ConstPtr msg)
{
    // Data for motion
    geometry_msgs::Twist twist;
    bool front_obstacle = false, back_obstacle = false;

    // Detecting whether there is an object within the area of the laser
    for(int i=0;i < msg->ranges.size(); i++)
        if(msg->ranges[i] <= (msg->range_max - 0.1))
        {
            if(i < 6 || i > (msg->ranges.size() - 6))
                front_obstacle = true;
            else
                back_obstacle = true;
        }

    // Marco's movement based on the current state and where the obstacle is
    switch(mstate)
    {
        case approaching:
            twist.linear.x = 0.5;
            pub.publish(twist);
            if(front_obstacle)
                mstate = talking;
            break;
        case talking:
            twist.linear.x = 0.0;
            pub.publish(twist);
            mstate = goingaway;
            ros::Duration(1).sleep();
            break;
        default:
            if(back_obstacle && !front_obstacle)
                mstate = approaching;
            else if(back_obstacle && front_obstacle)
                mstate = talking;
            else
            {
                twist.linear.x = -0.5;
                pub.publish(twist);
            }
    }
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

    ros::spin();

    return 0;
}
