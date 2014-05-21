#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <vector>

using namespace std;

geometry_msgs::Pose fposition; // Ferdi position
geometry_msgs::Twist twist;
ros::Publisher pub;

void fpose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    fposition = msg->pose;
    ROS_INFO("Ferdi x: %f, y: %f, w: %f", fposition.position.x, fposition.position.y, fposition.orientation.w);
}

void mpose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    geometry_msgs::Pose mposition = msg->pose;
    ROS_INFO("Marco x: %f, y: %f, w: %f", mposition.position.x, mposition.position.y, mposition.orientation.w);

    if(mposition.position.x <= -2.0 && mposition.position.x >= -2.1)
        twist.linear.x = -0.5;
    else if(mposition.position.x >= 0.9 && mposition.position.x <= 1.0)
        twist.linear.x = 0.5;

    pub.publish(twist);
}

void lpose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    geometry_msgs::Pose lposition = msg->pose;
    ROS_INFO("Lenka x: %f, y: %f, w: %f", lposition.position.x, lposition.position.y, lposition.orientation.w);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"HumanAI");
    ros::NodeHandle n;

    pub = n.advertise<geometry_msgs::Twist>("marco/mmotion",1000);

//    twist.linear.x = 0.5;
//    pub.publish(twist);

    ros::Subscriber subfpose = n.subscribe("ferdi/fpose",1000,fpose);
    ros::Subscriber submpose = n.subscribe("marco/mpose",1000,mpose);
    ros::Subscriber sublpose = n.subscribe("lenka/lpose",1000,lpose);
    ros::spin();

    return 0;
}
