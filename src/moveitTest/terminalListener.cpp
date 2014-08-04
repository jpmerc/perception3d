#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "/terminal_listener");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String> ("/terminal_listener", 1);

    while(ros::ok())
    {
        std::string input;
        std::cin >> input;
        std_msgs::String output;
        output.data = input;
        pub.publish(output);
        ros::spinOnce();
    }

    return 0;
}
