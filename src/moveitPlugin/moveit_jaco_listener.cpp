#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

/*
  This is the plugin to communicate with the move_group node.
  The move_group node is the core of moveit.  It's where the motion planning is computed and where all the
  data is received.
  It takes a PoseStamped message and it calls the moveGroup interface.
  We need this beacause the moveGroup need to be in its own queue.
  */

using namespace std;


void callBack(geometry_msgs::PoseStampedConstPtr p_input)
{
    moveit::planning_interface::MoveGroup group("arm");
    group.setPoseReferenceFrame(std::string("root"));
    group.setEndEffectorLink(std::string("jaco_link_hand"));
    group.setNumPlanningAttempts(5);


   //     group.setPlannerId("RRTstarkConfigDefault");

    /*  Planner similar to EST but expands two trees from the start and goal nodes.
        For this reason, the solutions found by the planner follow two diﬀerent trajectories.
        The ﬁrstone escapes from the start and the other approachs to the goal. */

    //group.setPlannerId("SBLkConfigDefault");

    /*  Planner used by default. This planner ﬁrst discretizes the workspace in feasible blocks (similar to octomap)
        distributes nodes in this zone and expands a tree from the start node. It’s a planner that usually works well */
    group.setPlannerId("KPIECEkConfigDefault");

    /*  Planner that expands a tree from the start node verifying the feasibility of the
        nodes in every expansion. So the diﬀerence with the KPIECE planner is that in
        KPIECE the veriﬁcation of the feasibility it’s made before the expansion of the whole
        tree. It’s good for scenes where there are narrow paths to achieve the goal node.       */
//     group.setPlannerId("ESTkConfigDefault");

    /*  planner totally random. For this reason, if there is one solution or it isn’t easy
        to ﬁnd a solution */


    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End Effector frame: %s", group.getEndEffectorLink().c_str());

    ros::NodeHandle node_handle;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;


    moveit::planning_interface::MoveGroup::Plan myPlan;
    group.setStartStateToCurrentState();
    group.setPoseTarget(*p_input);
    group.setPlanningTime(10.0);

//    std::cout << "IT IS NOW TIME TO PLAN!" << std::endl;

//    geometry_msgs::Pose pose_target = group.getCurrentPose("jaco_link_hand").pose;
//    geometry_msgs::Pose pose_target2 = p_input->pose;
//    std::vector<geometry_msgs::Pose> waypoints;
//    waypoints.push_back(pose_target);
//    waypoints.push_back(pose_target2);
//    moveit_msgs::RobotTrajectory trajectory_msg;
//    double fraction = group.computeCartesianPath(waypoints, 0.01, 0, trajectory_msg, true);
//    robot_trajectory::RobotTrajectory robot_trajectory(group.getCurrentState()->getRobotModel(),"arm");
//    robot_trajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);


//    trajectory_processing::IterativeParabolicTimeParameterization iptp;
//    bool success = iptp.computeTimeStamps(robot_trajectory);
//    robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);

//    myPlan.trajectory_ = trajectory_msg;

    bool success = group.plan(myPlan);

    cout << "Size of plan : " << myPlan.trajectory_.joint_trajectory.points.size() << endl;
    cout << "Plan :" << endl << myPlan.trajectory_.joint_trajectory << endl;

    ROS_INFO("Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = myPlan.start_state_;
    display_trajectory.trajectory.push_back(myPlan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    if(success)
    {
        std::cout << "The plan worked!" << std::endl;
        //group.move();
    }
    else
    {
        std::cout << "The plan failed!" << std::endl;
    }

    //cout << "The fraction score was : " << fraction << endl;

}

void addObstacleBehindJaco(){
    ros::NodeHandle nh;
    ros::Rate r(3);
    moveit::planning_interface::MoveGroup group("arm");
    ros::Publisher collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);

    // ADD OBSTACLE BEHIND JACO
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();
    collision_object.id = "wall_behind";

    // Define a box to add to the world
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 3.0;
    primitive.dimensions[1] = 0.3;
    primitive.dimensions[2] = 3.0;

    // A pose for the box (specified relative to frame_id root)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.0;
    box_pose.position.y =  0.45;
    box_pose.position.z =  1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    r.sleep();
    collision_object.operation = collision_object.REMOVE;
    r.sleep();
    collision_object_publisher.publish(collision_object);
    r.sleep();
    collision_object.operation = collision_object.ADD;
    r.sleep();
    collision_object_publisher.publish(collision_object);
    r.sleep();

}


int main(int argc, char** argv)
{


    ros::init(argc,argv,"moveit_jaco_listener");

    ros::NodeHandle nh;

    // Add an obstacle behind jaco arm to prevent collision with wheelchair user
    addObstacleBehindJaco();

    // Add an obstacle over jaco
    addObstacleBehindJaco();

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
