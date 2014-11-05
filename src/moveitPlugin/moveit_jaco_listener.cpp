#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPlanningScene.h>


/*
  This is the plugin to communicate with the move_group node.
  The move_group node is the core of moveit.  It's where the motion planning is computed and where all the
  data is received.
  It takes a PoseStamped message and it calls the moveGroup interface.
  We need this beacause the moveGroup need to be in its own queue.
  */

using namespace std;
shape_msgs::SolidPrimitive shape_;
geometry_msgs::Pose pose_;
ros::ServiceClient client_get_scene_;
ros::Publisher planning_scene_diff_publisher_;
void findBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc, shape_msgs::SolidPrimitive &shape, geometry_msgs::Pose &pose);
void modifyACM();


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
//    group.setPlannerId("KPIECEkConfigDefault");

    /*  Planner that expands a tree from the start node verifying the feasibility of the
        nodes in every expansion. So the diﬀerence with the KPIECE planner is that in
        KPIECE the veriﬁcation of the feasibility it’s made before the expansion of the whole
        tree. It’s good for scenes where there are narrow paths to achieve the goal node.       */
    //     group.setPlannerId("ESTkConfigDefault");

    /*  planner totally random. For this reason, if there is one solution or it isn’t easy
        to ﬁnd a solution */

    group.setPlannerId("RRTConnectkConfigDefault");

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
    cout << "Planning frame = " << group.getPlanningFrame() << endl;
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



void object_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc){
    ros::NodeHandle nh;
    ros::Rate r(3);
    shape_msgs::SolidPrimitive shape;
    geometry_msgs::Pose pose;

    findBoundingBox(pc, shape, pose);
//    cout << "Callback is done!" << endl;



    // Check if the object is the same that is already included in moveIt. If it is, then we do not refresh it.
    bool isTheSameObjectThanLastOne = false;

    if(shape.dimensions.size() >= 3 && shape_.dimensions.size() >= 3){
       if(shape.dimensions.at(0) == shape_.dimensions.at(0) && shape.dimensions.at(1) == shape_.dimensions.at(1) && shape.dimensions.at(2) == shape_.dimensions.at(2)){
           if(pose.position.x == pose_.position.x && pose.position.y == pose_.position.y && pose.position.z == pose_.position.z){
               isTheSameObjectThanLastOne = true;
           }
       }
    }

    shape_ = shape;
    pose_ =  pose;
    //    cout << "X : [ " << x_min  << " : " << x_max << " ] " <<  "Y : [ " << y_min  << " : " << y_max << " ] " << "Z : [ " << z_min  << " : " << z_max << " ] " << endl;


    if(!isTheSameObjectThanLastOne){
        ros::Publisher collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "/root";
        collision_object.id = "bounding_box";
        collision_object.primitives.push_back(shape);
        collision_object.primitive_poses.push_back(pose);

        r.sleep();
        collision_object.operation = collision_object.REMOVE;
        r.sleep();
        collision_object_publisher.publish(collision_object);
        r.sleep();
        collision_object.operation = collision_object.ADD;
        r.sleep();
        collision_object_publisher.publish(collision_object);
        r.sleep();

        // Allow Collisions between some joints of Jaco and the object/obstacle to grasp in trajectory planning
        modifyACM();
    }

}

// Inspired from https://groups.google.com/forum/#!topic/moveit-users/EI73skgnGVk
void modifyACM(){

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

    bool object_in_world = false;
    while(!object_in_world){
        cout << "waiting for box to appear..." << endl;
        if (client_get_scene_.call(srv))
        {
            for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i)
            {
                if (srv.response.scene.world.collision_objects[i].id == "bounding_box")
                    object_in_world = true;
            }
        }
    }

    moveit_msgs::PlanningScene currentScene;
    moveit_msgs::PlanningScene newSceneDiff;

    moveit_msgs::GetPlanningScene scene_srv;
    scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

    if(!client_get_scene_.call(scene_srv)){
        cout << "Failed to call service /get_planning_scene" << endl;
    }

    else{
        cout << "Initial Scene !" << endl;
        currentScene = scene_srv.response.scene;
        moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;
        cout << "size of acm_entry_names before " << currentACM.entry_names.size() << endl;
        cout << "size of acm_entry_values before " << currentACM.entry_values.size() << endl;
        cout << "size of acm_entry_values[0].entries before " << currentACM.entry_values[0].enabled.size() << endl;

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        planning_scene::PlanningScene planning_scene(kinematic_model);
        collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();


        acm.setEntry("bounding_box",false);
        vector<string> allowedLinksToCollideWithObjectToGrasp;
        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_1");
        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_tip_1");
        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_2");
        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_tip_2");
        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_3");
        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_tip_3");
        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_hand");
        acm.setEntry("bounding_box",allowedLinksToCollideWithObjectToGrasp,true);
        acm.getMessage(currentACM);


        newSceneDiff.is_diff = true;
        newSceneDiff.allowed_collision_matrix = currentACM;

        planning_scene_diff_publisher_.publish(newSceneDiff);


        // ------------------------------------------------------------------
        //DEBUG : Check if it has worked
//        if(!client_get_scene_.call(scene_srv)){
//            cout << "Failed to call service /get_planning_scene" << endl;
//        }

//        else{
//            cout << "Modified Scene !" << endl;
//            currentScene = scene_srv.response.scene;
//            moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;


//            for(int i = 0; i < currentACM.entry_names.size(); i++){
//                string entryName = currentACM.entry_names.at(i);
//                cout << "Entry : " << entryName << endl;
//                cout << "Value : " << currentACM.entry_values.at(i) << endl;
//            }

//            cout << "size of acm_entry_names after " << currentACM.entry_names.size() << endl;
//            cout << "size of acm_entry_values after " << currentACM.entry_values.size() << endl;
//            cout << "size of acm_entry_values[0].entries after " << currentACM.entry_values[0].enabled.size() << endl;
//        }

    }

}

// Find a bounding box around the object to grasp (pc)
// Outputs the shape and the pose of the bounding box
// Inspired from https://github.com/unboundedrobotics/ubr1_preview/blob/master/ubr1_grasping/src/shape_extraction.cpp
void findBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc, shape_msgs::SolidPrimitive &shape, geometry_msgs::Pose &pose){

    ros::NodeHandle nh;
    ros::Rate r(3);
    double x_min =  9999.0;
    double x_max = -9999.0;
    double y_min =  9999.0;
    double y_max = -9999.0;
    double z_min =  9999.0;
    double z_max = -9999.0;

    for(int i = 0; i < pc->size(); i++){
        double pc_x = pc->at(i).x;
        double pc_y = pc->at(i).y;
        double pc_z = pc->at(i).z;

        if(pc_x < x_min) x_min = pc_x;
        if(pc_y < y_min) y_min = pc_y;
        if(pc_z < z_min) z_min = pc_z;

        if(pc_x > x_max) x_max = pc_x;
        if(pc_y > y_max) y_max = pc_y;
        if(pc_z > z_max) z_max = pc_z;
    }

    pose.position.x = (x_min + x_max)/2.0;
    pose.position.y = (y_min + y_max)/2.0;
    pose.position.z = (z_min + z_max)/2.0;
    pose.orientation.w = 1.0;

    shape.type = shape.BOX;
    shape.dimensions.push_back(x_max-x_min);
    shape.dimensions.push_back(y_max-y_min);
    shape.dimensions.push_back(z_max-z_min);

}

int main(int argc, char** argv)
{


    ros::init(argc,argv,"moveit_jaco_listener");

    ros::NodeHandle nh;



    //Add an obstacle behind jaco to reduce its workspace (do not go too far behind --> kinect is there)
    addObstacleBehindJaco();

    ros::CallbackQueue queue;
    ros::SubscribeOptions options = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/jaco_command",1, boost::bind(&callBack,_1), ros::VoidPtr(), &queue);
    ros::Subscriber subA = nh.subscribe(options);
    ros::AsyncSpinner spinner(0, &queue);
    spinner.start();

    client_get_scene_ = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    planning_scene_diff_publisher_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    ros::Subscriber subB = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/grasp_object", 1, object_callback);

    ros::spin();

    return 0;
}
