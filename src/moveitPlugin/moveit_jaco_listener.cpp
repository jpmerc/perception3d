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
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/CollisionObject.h>


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
ros::Publisher movement_status_publisher_;



moveit_msgs::Grasp tryMoveItPickMessage(geometry_msgs::PoseStampedConstPtr p_input){

    moveit_msgs::Grasp grasp;

    grasp.grasp_pose = *p_input;

    grasp.pre_grasp_approach.direction.vector.z = 1.0;
    grasp.pre_grasp_approach.direction.header.frame_id = "jaco_link_hand";
    grasp.pre_grasp_approach.min_distance = 0.05;
    grasp.pre_grasp_approach.desired_distance = 0.1;
    grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();

    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.min_distance = 0.05;
    grasp.post_grasp_retreat.desired_distance = 0.1;
    grasp.post_grasp_retreat.direction.header.frame_id = "jaco_link_hand";
    grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();

   return grasp;
}

bool acceptOrRejectTrajectory(){
    cout << "Do you accept the trajectory planned by MoveIt! ? (y or n) " << endl;

    string accept = "";
    cin >> accept;

    char c = accept[0];
    if(c == 'y') return true;
    else return false;

}

void callBack(geometry_msgs::PoseStampedConstPtr p_input)
{


    //moveit_msgs::Grasp grasp = tryMoveItPickMessage(p_input);
    // Publish TF TO check if target is good!
//    static tf::TransformBroadcaster br;
//    tf::Transform trans;
//    trans.setOrigin(tf::Vector3(p_input->pose.position.x, p_input->pose.position.y, p_input->pose.position.z));
//    tf::Quaternion quat;
//    tf::quaternionMsgToTF(p_input->pose.orientation, quat);
//    trans.setRotation(quat);
//    cout << "Sending transform!" << endl;

//    br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "root", "moveit_target_pose"));
//    sleep(1);
//    br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "root", "moveit_target_pose"));
//    sleep(1);
//    br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "root", "moveit_target_pose"));
//    sleep(1);

    moveit::planning_interface::MoveGroup group("arm");
    group.setPoseReferenceFrame(std::string("root"));
    group.setEndEffectorLink(std::string("jaco_link_hand"));
    group.setNumPlanningAttempts(10);
    group.setPlannerId("RRTConnectkConfigDefault");
    //group.setStartStateToCurrentState();
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = p_input->pose.position.x;
    target_pose1.position.y = p_input->pose.position.y;
    target_pose1.position.z = p_input->pose.position.z;
    target_pose1.orientation.x = p_input->pose.orientation.x;
    target_pose1.orientation.y = p_input->pose.orientation.y;
    target_pose1.orientation.z = p_input->pose.orientation.z;
    target_pose1.orientation.w = p_input->pose.orientation.w;


    cout << "x : " << p_input->pose.orientation.x << endl;
    cout << "y : " << p_input->pose.orientation.y << endl;
    cout << "z : " << p_input->pose.orientation.z << endl;
    cout << "w : " << p_input->pose.orientation.w << endl;

   // group.setPoseTarget(*p_input,std::string("jaco_link_hand"));
    group.setPoseTarget(target_pose1, std::string("jaco_link_hand"));
    group.setPlanningTime(10.0);
    group.setWorkspace(-1, -1.5 , 0.1, 1, 0.4, 1.2);
   // group.setWorkspace(-2,-2,-2,2,2,2);
    //group.setGoalPositionTolerance(0.10);
    group.setGoalTolerance(0.10);


    ros::NodeHandle node_handle;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit::planning_interface::MoveGroup::Plan myPlan;


//    // CHECK THE COLLISION WORLD AND ALLOWED COLLISION MATRIX JUST BEFORE PLANNING
//    moveit_msgs::GetPlanningScene srv;
//    srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

//    //cout << "waiting for box to appear..." << endl;
//    if (client_get_scene_.call(srv))
//    {
//        bool object_in_world = false;
//        while(!object_in_world){
//            for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i)
//            {
//                if (srv.response.scene.world.collision_objects[i].id == "bounding_box")
//                    object_in_world = true;
//            }
//        }
////        cout << "WORLD OBJECTS : " << endl;
////        for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i){
////            cout << srv.response.scene.world.collision_objects[i].id << endl;
////        }

//    }


    bool success = group.plan(myPlan);


    // PRINT TRAJECTORY POINTS
    // cout << "Size of plan : " << myPlan.trajectory_.joint_trajectory.points.size() << endl;
    // cout << "Plan :" << endl << myPlan.trajectory_.joint_trajectory << endl;

    // DISPLAY IN RVIZ
    display_trajectory.trajectory_start = myPlan.start_state_;
    display_trajectory.trajectory.push_back(myPlan.trajectory_);
    display_publisher.publish(display_trajectory);
    sleep(5.0);
    /* Sleep to give Rviz time to visualize the plan. */


    std_msgs::Bool moved_successfully;
    if(success)
    {
        std::cout << "The plan worked!" << std::endl;
        bool accept = acceptOrRejectTrajectory();

        // TO REMOVE WHEN FINISHED WITH TESTS
        sleep(10);

        if(accept) {
//            group.move();
            moved_successfully.data = true;
        }
        else{
            moved_successfully.data = false;
        }
    }
    else
    {
        std::cout << "The plan failed!" << std::endl;
        moved_successfully.data = false;
    }

    //cout << "The fraction score was : " << fraction << endl;

    movement_status_publisher_.publish(moved_successfully);

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


//        collision_object.operation = collision_object.ADD;

//        ros::NodeHandle node_handle;
//        ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);


//        ros::WallDuration sleep_time(0.5);
//        moveit_msgs::PlanningScene planning_scene;
////        planning_scene.world.collision_objects.push_back(collision_object);
////        planning_scene.is_diff = true;
//        planning_scene_diff_publisher.publish(planning_scene);
//        sleep_time.sleep();



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
        //cout << "waiting for box to appear..." << endl;
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
//        cout << "size of acm_entry_names before " << currentACM.entry_names.size() << endl;
//        cout << "size of acm_entry_values before " << currentACM.entry_values.size() << endl;
//        cout << "size of acm_entry_values[0].entries before " << currentACM.entry_values[0].enabled.size() << endl;

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        planning_scene::PlanningScene planning_scene(kinematic_model);
        collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();


//        acm.setEntry("bounding_box",false);
//        vector<string> allowedLinksToCollideWithObjectToGrasp;
//        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_1");
//        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_tip_1");
//        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_2");
//        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_tip_2");
//        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_3");
//        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_finger_tip_3");
//        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_hand");
//        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_small_ring_cover_3");

//        //TEST
////        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_3");
////        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_4");
////        allowedLinksToCollideWithObjectToGrasp.push_back("jaco_link_5");

//        acm.setEntry("bounding_box",allowedLinksToCollideWithObjectToGrasp,true);

        acm.setEntry("bounding_box",true);
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
    //addObstacleBehindJaco();

    ros::CallbackQueue queue;
    ros::SubscribeOptions options = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/jaco_command",1, boost::bind(&callBack,_1), ros::VoidPtr(), &queue);
    ros::Subscriber subA = nh.subscribe(options);
    ros::AsyncSpinner spinner(0, &queue);
    spinner.start();

    client_get_scene_ = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    planning_scene_diff_publisher_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    movement_status_publisher_ = nh.advertise<std_msgs::Bool>("/jaco_arm/moveit_movement_status",1);

    //ros::Subscriber subB = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/grasp_object", 1, object_callback);

    ros::spin();
    spinner.stop();

    return 0;
}
