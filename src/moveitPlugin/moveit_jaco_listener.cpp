#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/*
  This is the plugin to communicate with the move_group node.
  The move_group node its core of moveit.  Its where the planning is compute and where all the
  datas is received.
  It take a PoseStamped message and it call the moveGroup interface.
  We need this beacause the moveGroup need to be in its own queue.
  */




void callBack(geometry_msgs::PoseStampedConstPtr p_input)
{
    moveit::planning_interface::MoveGroup group("arm");

    moveit::planning_interface::MoveGroup::Plan myPlan;
    group.setPoseTarget(*p_input);
    bool success = group.plan(myPlan);
    if(success)
    {
        std::cout << "the plan work" << std::endl;
        group.move();
    }
    else
    {
        std::cout << "the plan fail" << std::endl;
    }

}


int main(int argc, char** argv)
{


    ros::init(argc,argv,"moveit_jaco_listener");

    ros::NodeHandle nh;


    ros::CallbackQueue queue;
    ros::SubscribeOptions options = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/jaco_command",
                                                                                    1,
                                                                                    boost::bind(&callBack,_1),
                                                                                    ros::VoidPtr(),
                                                                                    &queue);

    ros::Subscriber subA = nh.subscribe(options);
    ros::AsyncSpinner spinner(0, &queue);
    spinner.start();
    ros::spin();


    return 0;
}
