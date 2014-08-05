#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <jaco_custom.h>


std::string MESSAGE;

void callBack(std_msgs::String p_input)
{
    MESSAGE = p_input.data;
}



int main(int argc, char** argv)
{


    ros::init(argc,argv,"test_moveit_jaco");

    ros::NodeHandle nh;
    ros::Publisher pub;

    pub = nh.advertise<sensor_msgs::PointCloud2>("robot_vision", 1);

    JacoCustom ja(nh);

    ros::Subscriber sub = nh.subscribe("/terminal_listener", 1, callBack);

    while(ros::ok())
    {
        ros::spinOnce();
        if(MESSAGE != "")
        {
            double distance = atof(MESSAGE.c_str());
            ja.jeanMoveup(distance);
            MESSAGE = "";
        }
    }

    return 0;
}

