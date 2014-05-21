#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

// ROS Publisher
ros::Publisher pub;
// mpos = marco's position, the guy who initially stands near the right table
geometry_msgs::Pose mpos;
// a flag to mark whether the previous movement was stopped by an occlusion.
bool past_move_flag;

// The following three functions show the position for each human.
// Positions for the third guy who stands near the couch
void fpose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    geometry_msgs::Pose pos = msg->pose;
    ROS_INFO("Ferdi x: %f, y: %f, w: %f", pos.position.x, pos.position.y, pos.orientation.w);
}

// Positions for the second guy
void mpose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    mpos = msg->pose;
    ROS_INFO("Marco x: %f, y: %f, w: %f", mpos.position.x, mpos.position.y, mpos.orientation.w);
}

// Positions for the first guy
void lpose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    geometry_msgs::Pose lpos = msg->pose;
    ROS_INFO("Lenka x: %f, y: %f, w: %f", lpos.position.x, lpos.position.y, lpos.orientation.w);
}

// ros::Duration(0.25).sleep(); // to sleep 0.25 seconds
// A function to update the movement of the second guy based on laser sensor
void laser(const sensor_msgs::LaserScan::ConstPtr msg)
{
    bool move_flag;
    geometry_msgs::Twist twist;

    // Detecting whether there is an object within the half of the maximum range of the laser
    for(int i=0;i < msg->ranges.size(); i++)
        if(msg->ranges[i] <= (msg->range_max)/1.2)
        {
          move_flag = false;
          break;
        }
        else move_flag = true;

    // Updating the movement of the second guy
    // if the position of the second guy is near to the first guy. Move backward
    if(mpos.position.x <= -1.9)
        twist.linear.x = -0.5;
    // if the previous movement was stop, and now there is no occlusion and
    // the position of the second guy is not really near to the first guy, then move forward
    else if(mpos.position.x >= 1.0 ||
            (mpos.position.x > -1.9 && mpos.position.x < 1.0 && move_flag && !past_move_flag))
        twist.linear.x = 0.5;
    // If there is an occlusion, stop.
    else if(!move_flag)
        twist.linear.x = 0.0;

    // set the current flag as the old flag
    past_move_flag = move_flag;

    // publish the movement
    pub.publish(twist);
}

int main(int argc, char **argv)
{
    // ros initialization
    ros::init(argc,argv,"HumanAI");
    ros::NodeHandle n;

    // publishing topic for the second guy movement
    pub = n.advertise<geometry_msgs::Twist>("marco/mmotion",1000);
    // subscribing to three human positions
    ros::Subscriber subfpose = n.subscribe("ferdi/fpose",1000,fpose);
    ros::Subscriber submpose = n.subscribe("marco/mpose",1000,mpose);
    ros::Subscriber sublpose = n.subscribe("lenka/lpose",1000,lpose);
    // subscribing to laser information
    ros::Subscriber sublaser = n.subscribe("marco/laser",1000,laser);
    ros::spin();

    return 0;
}
