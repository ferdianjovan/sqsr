#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <boost/lexical_cast.hpp>
#include <vector>
#include <cmath>

using namespace std;

ros::Publisher motion_pub;
geometry_msgs::Point pioneer_position;
bool found;
vector<string> object_name;
vector<float> object_position;

// Obtain information about laser and together with object_position and object_name,
// it tries to locate the human which is able to be moved by keyboard.
void laser_Callback(const sensor_msgs::LaserScan::ConstPtr msg)
{
    geometry_msgs::Twist twist;
    bool obstacle_left, obstacle_right;
    found = false;

    for(unsigned i = 0; i < object_name.size(); ++i)
    {
        if(object_name[i].compare("Lenka") == 0)
        {
            if((abs(object_position[i*2] - pioneer_position.x) <= 1.2) && (abs(object_position[(i*2)+1] - pioneer_position.y) <= 1.2))
            {
                found = true;
                break;
            }
        }
    }

    if(!found)
    {
        for(int i = 0; i < msg->ranges.size(); i++)
            if(msg->ranges[i] < msg->range_max)
                if(i < 4) obstacle_right = true;
                else obstacle_left = true;

        if(!obstacle_left && !obstacle_right)
            twist.linear.x = 0.25;
        else if(!obstacle_left && obstacle_right)
            twist.angular.z = 2.0;
        else if(obstacle_left && !obstacle_right)
            twist.angular.z = -2.0;
        else
        {
            twist.linear.x = -0.25;
            twist.angular.z = -0.75;
        }
    }
    else
    {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    }

    motion_pub.publish(twist);
}

void pioneer_position_Callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    pioneer_position = msg->pose.position;
    ROS_INFO("Pioneer position x: %f, y: %f", pioneer_position.x, pioneer_position.y);
}

// Obtain information about objects in the scene (names and x-y positions)
void camera_Callback(const std_msgs::String::ConstPtr msg)
{
    vector<string> object_name_temp;
    vector<float> object_position_temp;

    string::size_type name_index = msg->data.find("\"name\"");

    while(name_index != string::npos)
    {
        string::size_type first_double_quote = msg->data.find("\"", name_index+7);
        string::size_type second_double_quote = msg->data.find("\"", first_double_quote+1);

        object_name_temp.push_back(msg->data.substr(first_double_quote+1, (second_double_quote - 1) - first_double_quote));
        string::size_type position_index = msg->data.find("\"position\"",second_double_quote+1);

        first_double_quote = msg->data.find("[", position_index+10);
        second_double_quote = msg->data.find(",", first_double_quote+1);

        object_position_temp.push_back(boost::lexical_cast<float>(msg->data.substr(first_double_quote+1, (second_double_quote-1) - first_double_quote)));

        first_double_quote = second_double_quote+1;
        second_double_quote = msg->data.find(",", first_double_quote+1);

        object_position_temp.push_back(boost::lexical_cast<float>(msg->data.substr(first_double_quote+1, (second_double_quote-1) - first_double_quote)));

        name_index = msg->data.find("\"name\"", second_double_quote+1);
    }

    object_name = object_name_temp;
    object_position = object_position_temp;
    //ROS_INFO("%d, %d", object_name_temp.size(), object_position_temp.size());

//    for(unsigned i=0; i < object_name.size(); i++)
//    {
//        ROS_INFO(object_name.at(i).data());
//        ROS_INFO("x: %f, y: %f", object_position.at(i*2), object_position.at((i*2)+1));
//    }
}

int  main(int argc, char **argv)
{
    ros::init(argc,argv,"human_searcher");
    ros::NodeHandle n;

    motion_pub = n.advertise<geometry_msgs::Twist>("pioneer/piomotion",1000);

    ros::Subscriber laser_sub = n.subscribe("pioneer/piolaser",1000,laser_Callback);
    ros::Subscriber camera_sub = n.subscribe("pioneer/piocamera",1000,camera_Callback);
    ros::Subscriber position_sub = n.subscribe("pioneer/piopose",1000,pioneer_position_Callback);

    ros::spin();

    return 0;
}

// Standard ROS to use time!!
////Get the time and store it in the time variable.
//ros::Time time = ros::Time::now()
////Wait a duration of one second.
//ros::Duration d = ros::Duration(1, 0);
//d.sleep();
