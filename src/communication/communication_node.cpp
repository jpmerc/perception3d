#include <ros/ros.h>
#include <communication.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "communication_node");

    ros::NodeHandle nh;
    Communication* obj_communication_ptr(new Communication(nh));

    ros::Subscriber sub = nh.subscribe("/image_coordinate_rgb", 1, &Communication::callback_android_listener,obj_communication_ptr);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 1;
}
