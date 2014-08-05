#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <jaco_custom.h>


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

    //ros::Subscriber sub = nh.subscribe("/terminal_listener", 1, callBack);
/*
    while(ros::ok())
    {
        ros::spinOnce();
        std::cout << MESSAGE << std::endl;
        if(MESSAGE.size() != 0)
        {
            double distance = atof(MESSAGE.c_str());
            ja.jeanMoveup(distance);
            std::cout << "Distance " << distance << std::endl;
            MESSAGE = "";
        }
        ros::Rate r(30);
                r.sleep();

    moveit::planning_interface::MoveGroup group("arm");

    moveit::planning_interface::MoveGroup::Plan myPlan;
    group.setRandomTarget ();
    bool success = group.plan(myPlan);
    if(success)
    {
        std::cout << "the plan work" << std::endl;
        //group.move();
    }
        else{
            std::cout << "the plan fail" << std::endl;
        }

    ros::shutdown();
*/
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
