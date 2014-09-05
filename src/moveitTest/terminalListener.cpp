#include <ros/ros.h>
#include <std_msgs/String.h>

/*
  Its a simple therminal listener to debug think.  It will publish a string that contain the line in
  the terminal.
  */


int main(int argc, char** argv)
{
    ros::init(argc, argv, "terminal_listener");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String> ("/terminal_listener", 1);

    while(ros::ok())
    {
        std::string input;
        std::cout << "Enter your command : " << std::endl;
        std::cin >> input;
        if(input == "q" || input == "exit" || input == "quit"){
            break;
        }
        std_msgs::String output;
        output.data = input;
        pub.publish(output);
       // ros::spinOnce();
    }

    return 0;
}
