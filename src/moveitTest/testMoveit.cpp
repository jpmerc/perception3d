#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <jaco_custom.h>

/*
  A test file that call JacoCustom to test the moveit pipeline.  It listen to terminal_listener
  topic.  So you can move the arm with the terminal.
  */



std::string MESSAGE;

void callBack(std_msgs::String p_input)
{
    MESSAGE = p_input.data;
    std::cout << "Message " << MESSAGE << std::endl;
}

void callBack2(std_msgs::StringConstPtr p_input, JacoCustom* p_ja)
{
    double distance = atof(p_input->data.c_str());
    p_ja->jeanMoveup(distance);
}


int main(int argc, char** argv)
{


    ros::init(argc,argv,"test_moveit_jaco");

    ros::NodeHandle nh;
    ros::Publisher pub;
    //ros::AsyncSpinner spinner(1);
    //  spinner.start();

    pub = nh.advertise<sensor_msgs::PointCloud2>("robot_vision", 1);

    JacoCustom ja(nh);


    ros::CallbackQueue queue;
    ros::SubscribeOptions options = ros::SubscribeOptions::create<std_msgs::String>("/terminal_listener",
                                                                                    1,
                                                                                    boost::bind(&callBack2,_1, &ja),
                                                                                    ros::VoidPtr(),
                                                                                    &queue);

    ros::AsyncSpinner spinner(0, &queue);
    spinner.start();
    ros::spin();


    return 0;
}
