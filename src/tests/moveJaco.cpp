#include <ros/ros.h>
#include <stdlib.h>
#include <sstream>
#include <stdio.h>
#include <fileAPI.h>
#include <object_recognition.h>
#include <jaco_custom.h>

using namespace std;


JacoCustom* JACO_PTR;
ros::CallbackQueue jaco_callbacks;



void callbackThread(){
    ros::NodeHandle n;ros::Rate r(5);
    while(n.ok()){
        jaco_callbacks.callAvailable(ros::WallDuration(0));
        r.sleep();
    }
}




int main (int argc, char** argv){

    // Initialize ROS
    ros::init (argc, argv, "snapshot");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    JACO_PTR = new JacoCustom(n);


    // Different Callback Queue for Jaco Callbacks (position)
    const std::string arm_topic = "/jaco_arm_driver/out/tool_position";
    const std::string fingers_topic = "/jaco_arm_driver/out/finger_position";
    ros::SubscribeOptions fingers = ros::SubscribeOptions::create<jaco_msgs::FingerPosition>(fingers_topic,1,boost::bind(&JacoCustom::fingers_position_callback,JACO_PTR,_1),ros::VoidPtr(),&jaco_callbacks);
    ros::Subscriber sub_f = n.subscribe(fingers);
    ros::SubscribeOptions arm = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(arm_topic,1,boost::bind(&JacoCustom::arm_position_callback,JACO_PTR,_1),ros::VoidPtr(),&jaco_callbacks);
    ros::Subscriber sub_a = n.subscribe(arm);
    boost::thread spin_thread(callbackThread);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.22209;
    pose.pose.position.y = -0.58097;
    pose.pose.position.z = 0.38685;
    pose.pose.orientation.x = -0.03459;
    pose.pose.orientation.y = 0.14507;
    pose.pose.orientation.z = 0.94773;
    pose.pose.orientation.w = -0.28206;
    pose.header.frame_id = "/root";

    JACO_PTR->moveitPlugin(pose);

    return 0;
}
