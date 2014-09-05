#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>

/*
  This is the plugin to communicate with the move_group node.
  The move_group node is the core of moveit.  It's where the motion planning is computed and where all the
  data is received.
  It takes a PoseStamped message and it calls the moveGroup interface.
  We need this beacause the moveGroup need to be in its own queue.
  */




void callBack(geometry_msgs::PoseStampedConstPtr p_input)
{
    moveit::planning_interface::MoveGroup group("arm");
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End Effector frame: %s", group.getEndEffectorLink().c_str());

    ros::NodeHandle node_handle;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit::planning_interface::MoveGroup::Plan myPlan;
    group.setPoseTarget(*p_input);

    std::cout << "IT IS NOW TIME TO PLAN!" << std::endl;
    bool success = group.plan(myPlan);

    // TO REMOVE
    sleep(5);

    if(success)
    {
        std::cout << "The plan worked!" << std::endl;
        group.move();
    }
    else
    {
        std::cout << "The plan failed!" << std::endl;
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
