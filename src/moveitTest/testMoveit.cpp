#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <jaco_custom.h>

ros::NodeHandle nh;
ros::Publisher pub;


void callBack(std_msgs::String p_input)
{
    JacoCustom ja(nh);

    std::string string = p_input.data;

    double distance = atof(string.c_str());

    ja.jeanMoveup(distance);


}



int main(int argc, char** argv)
{


    ros::init(argc,argv,"test_moveit_jaco");

    pub = nh.advertise<sensor_msgs::PointCloud2>("robot_vision", 1);

    ros::Subscriber sub = nh.subscribe("/terminal_listener", 1, callBack);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}

