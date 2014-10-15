#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <sstream>
#include <stdio.h>
#include <objectExtractor.h>
#include <communication.h>
#include <jaco_custom.h>
#include "std_msgs/String.h"

using namespace std;


int main (int argc, char** argv){

    // Initialize ROS
    ros::init (argc, argv, "moveJaco");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ros::Rate r(1);

    ros::Publisher moveitPublisher = n.advertise<geometry_msgs::PoseStamped>("jaco_command",1);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.22209;
    pose.pose.position.y = -0.58097;
    pose.pose.position.z = 0.38685;
    pose.pose.orientation.x = -0.03459;
    pose.pose.orientation.y = 0.14507;
    pose.pose.orientation.z = 0.94773;
    pose.pose.orientation.w = -0.28206;
    pose.header.frame_id = "/root";

    r.sleep();
    moveitPublisher.publish(pose);
    r.sleep();

    return 0;
}
