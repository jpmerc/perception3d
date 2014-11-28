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



//    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.0270004;
    pose.pose.position.y = -0.597257;
    pose.pose.position.z = 0.323911;
    tf::Quaternion tf_quat;
    tf_quat.setEulerZYX(0.0931827053428, 1.05136871338, 1.51835548878);
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(tf_quat, quat);
    pose.pose.orientation = quat;
    pose.header.frame_id = "/root";

    r.sleep();
    moveitPublisher.publish(pose);
    r.sleep();

    return 0;
}
