#include "jaco_custom.h"

using namespace std;

JacoCustom::JacoCustom(ros::NodeHandle &node)
{
    end_program = false;

    arm_is_stopped = false;
    check_arm_status = false;
    arm_is_stopped_counter = 0;

    fingers_are_stopped = false;
    check_fingers_status = false;
    fingers_are_stopped_counter = 0;

    moveitPublisher = node.advertise<geometry_msgs::PoseStamped>("/jaco_command",1);

    angular_publisher   = node.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);
    cartesian_publisher = node.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 1);

    cartesian_position_service_client = node.serviceClient<wpi_jaco_msgs::GetCartesianPosition>("jaco_arm/get_cartesian_position");

}


//void JacoCustom::arm_position_callback (const geometry_msgs::PoseStampedConstPtr& input_pose){

//    if(check_arm_status){
//        bool same_pose = is_same_pose(&arm_pose,input_pose);
//        if(same_pose){
//            arm_is_stopped_counter++;
//        }
//        if(arm_is_stopped_counter >= 5){
//            arm_is_stopped = true;
//        }
//    }

//    // Assign new value to arm pose
//    arm_mutex.lock();
//    arm_pose = *input_pose;
//    arm_mutex.unlock();

//    // Publish the topic data received to a TF
//    tool_position_tf.setOrigin(tf::Vector3(input_pose->pose.position.x,input_pose->pose.position.y,input_pose->pose.position.z));
//    tool_position_tf.setRotation(tf::Quaternion(input_pose->pose.orientation.x,input_pose->pose.orientation.y,input_pose->pose.orientation.z,input_pose->pose.orientation.w));
//    position_broadcaster.sendTransform(tf::StampedTransform(tool_position_tf,ros::Time::now(),"jaco_api_origin","jaco_tool_position"));

//}


//void JacoCustom::fingers_position_callback(const jaco_msgs::FingerPositionConstPtr& input_fingers){
//    if(check_fingers_status){
//        bool same_pose = is_same_pose(&fingers_pose,input_fingers); //create a new function
//        if(same_pose){
//            fingers_are_stopped_counter++;
//        }
//        if(fingers_are_stopped_counter >= 5){
//            fingers_are_stopped = true;
//        }
//    }

//    // Assign new value to arm pose
//    fingers_mutex.lock();
//    fingers_pose = *input_fingers;
//    fingers_mutex.unlock();

//    //cout << " 1 : " << input_fingers->Finger_1 << endl;

//}


void JacoCustom::joint_state_callback (const sensor_msgs::JointStateConstPtr& input_pose){

    if(check_fingers_status){
        bool same_pose = is_same_pose_fingers(*joint_state, *input_pose); //create a new function
        if(same_pose){
            fingers_are_stopped_counter++;
        }
        if(fingers_are_stopped_counter >= 5){
            fingers_are_stopped = true;
        }
    }

    if(check_arm_status){
        bool same_pose = is_same_pose_arm(*joint_state, *input_pose);
        if(same_pose){
            arm_is_stopped_counter++;
        }
        if(arm_is_stopped_counter >= 5){
            arm_is_stopped = true;
        }
    }


    joint_mutex.lock();
    joint_state = input_pose;
    joint_mutex.unlock();


    // Publish the topic data received to a TF
    wpi_jaco_msgs::GetCartesianPosition srv;
    if(cartesian_position_service_client.call(srv)){
        tool_position_tf.setOrigin(tf::Vector3(srv.response.pos.linear.x, srv.response.pos.linear.y ,srv.response.pos.linear.z));
        tf::Quaternion quat;
        quat.setEuler(srv.response.pos.angular.z, srv.response.pos.angular.y, srv.response.pos.angular.x);
        tool_position_tf.setRotation(quat);
        position_broadcaster.sendTransform(tf::StampedTransform(tool_position_tf,ros::Time::now(),"jaco_link_base","jaco_tool_position"));
    }
}

void JacoCustom::moveit_move_status_callback (const std_msgs::BoolConstPtr in_status){

    moveit_status_received = true;
    move_it_status = in_status->data;

}


//void JacoCustom::open_fingers(){
//    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> action_client("jaco/finger_joint_angles",true);
//    action_client.waitForServer();
//    jaco_msgs::SetFingersPositionGoal fingers = jaco_msgs::SetFingersPositionGoal();
//    fingers.fingers.finger1 = 0;
//    fingers.fingers.finger2 = 0;
//    fingers.fingers.finger3 = 0;
//    action_client.sendGoal(fingers);
//    wait_for_fingers_stopped();
//}

//void JacoCustom::close_fingers(){
//    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> action_client("jaco/finger_joint_angles",true);
//    action_client.waitForServer();
//    jaco_msgs::SetFingersPositionGoal fingers = jaco_msgs::SetFingersPositionGoal();
//    fingers.fingers.finger1 = 60;
//    fingers.fingers.finger2 = 60;
//    fingers.fingers.finger3 = 60;
//    action_client.sendGoal(fingers);
//    wait_for_fingers_stopped();
//}

void JacoCustom::open_fingers(){
    actionlib::SimpleActionClient<wpi_jaco_msgs::ExecuteGraspAction> action_client("jaco_arm/manipulation/grasp",true);
    action_client.waitForServer();
    wpi_jaco_msgs::ExecuteGraspGoal goal = wpi_jaco_msgs::ExecuteGraspGoal();
    goal.closeGripper = false;
    action_client.sendGoal(goal);
}

void JacoCustom::close_fingers(){
    actionlib::SimpleActionClient<wpi_jaco_msgs::ExecuteGraspAction> action_client("jaco_arm/manipulation/grasp",true);
    action_client.waitForServer();
    wpi_jaco_msgs::ExecuteGraspGoal goal = wpi_jaco_msgs::ExecuteGraspGoal();
    goal.closeGripper = true;
    action_client.sendGoal(goal);
}



//void JacoCustom::move_up(double distance){
//    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
//    action_client.waitForServer();
//    jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();

//    arm_mutex.lock();
//    pose_goal.pose = this->arm_pose;
//    pose_goal.pose.header.frame_id = "/jaco_api_origin";
//    pose_goal.pose.pose.position.z += distance;
//    arm_mutex.unlock();

//    action_client.sendGoal(pose_goal);
//    wait_for_arm_stopped();
//}

//void JacoCustom::moveToPoint(double x, double y, double z, double rotx, double roty, double rotz, double rotw){
//    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
//    action_client.waitForServer();
//    jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();

//    pose_goal.pose.header.frame_id = "/jaco_api_origin";
//    pose_goal.pose.pose.position.x = x;
//    pose_goal.pose.pose.position.y = y;
//    pose_goal.pose.pose.position.z = z;
//    pose_goal.pose.pose.orientation.x = rotx;
//    pose_goal.pose.pose.orientation.y = roty;
//    pose_goal.pose.pose.orientation.z = rotz;
//    pose_goal.pose.pose.orientation.w = rotw;
//    action_client.sendGoal(pose_goal);

//    wait_for_arm_stopped();
//}

//void JacoCustom::moveToPoint(tf::Transform tf_){
//    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
//    action_client.waitForServer();
//    jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();

//    pose_goal.pose.header.frame_id = "/jaco_api_origin";
//    pose_goal.pose.pose.position.x = tf_.getOrigin().getX();
//    pose_goal.pose.pose.position.y = tf_.getOrigin().getY();
//    pose_goal.pose.pose.position.z = tf_.getOrigin().getZ();
//    pose_goal.pose.pose.orientation.x = tf_.getRotation().getX();
//    pose_goal.pose.pose.orientation.y = tf_.getRotation().getY();
//    pose_goal.pose.pose.orientation.z = tf_.getRotation().getZ();
//    pose_goal.pose.pose.orientation.w = tf_.getRotation().getW();
//    action_client.sendGoal(pose_goal);

//    wait_for_arm_stopped();
//}


//geometry_msgs::PoseStamped JacoCustom::getArmPosition(){
//    arm_mutex.lock();
//    geometry_msgs::PoseStamped arm = arm_pose;
//    arm_mutex.unlock();
//    return arm;
//}

tf::StampedTransform JacoCustom::getArmPositionFromCamera(){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("camera_rgb_frame","jaco_link_hand",ros::Time(0),ros::Duration(1.0));
    listener.lookupTransform("camera_rgb_frame","jaco_link_hand",ros::Time(0),transform);
    return transform;
}


//jaco_msgs::FingerPosition JacoCustom::getFingersPosition(){
//    fingers_mutex.lock();
//    jaco_msgs::FingerPosition pose = fingers_pose;
//    fingers_mutex.unlock();
//    return pose;
//}

std::vector<double> JacoCustom::getFingersPosition(){
    std::vector<double> v;
    joint_mutex.lock();
    v.push_back(joint_state->position.at(6));
    v.push_back(joint_state->position.at(7));
    v.push_back(joint_state->position.at(8));
    joint_mutex.unlock();
    return v;
}

tf::StampedTransform JacoCustom::getGraspArmPosition(){
    // It is supposed that the fingers are open at the beginning and that the user is teaching the grasp
    // position when the fingers are closing

    std::vector<double> old_pose = getFingersPosition();
    std::vector<double> new_pose;

    // If the fingers are closed by more than threshold angle, open the fingers
    double treshold = 0.05;
    if(old_pose.at(0) > treshold || old_pose.at(1) > treshold || old_pose.at(2) > treshold){
        open_fingers();
    }

    bool cond = true;
    ros::Rate r(4);
    int closing_count = 0;
    double closed_threshold = 0.75;

    // Waits until the fingers are closed more than closed_threshold
    while(cond){
        r.sleep();
        new_pose = getFingersPosition();

        if(new_pose.at(0) >= closed_threshold || new_pose.at(1) >= closed_threshold || new_pose.at(2) >= closed_threshold){

            // Wait until the fingers are not closing anymore
            if(new_pose.at(0) > old_pose.at(0) || new_pose.at(1) > old_pose.at(1) || new_pose.at(2) > old_pose.at(2)){
                closing_count = 0;
            }

            else{
                closing_count++;
            }
        }

        if(closing_count >= 5){
            cond = false;
        }

        // save current values for future iterations
        old_pose = new_pose;
    }

    sleep(1);
    return getArmPositionFromCamera();
}


//bool JacoCustom::is_same_pose(geometry_msgs::PoseStamped* pose1, geometry_msgs::PoseStampedConstPtr pose2){
//    bool cond1 = pose1->pose.position.x == pose2->pose.position.x;
//    bool cond2 = pose1->pose.position.y == pose2->pose.position.y;
//    bool cond3 = pose1->pose.position.z == pose2->pose.position.z;
//    bool cond4 = pose1->pose.orientation.x == pose2->pose.orientation.x;
//    bool cond5 = pose1->pose.orientation.y == pose2->pose.orientation.y;
//    bool cond6 = pose1->pose.orientation.z == pose2->pose.orientation.z;
//    bool cond7 = pose1->pose.orientation.w == pose2->pose.orientation.w;

//    if(cond1 && cond2 && cond3 && cond4 && cond5 && cond6 && cond7){
//        return true;
//    }

//    else{
//        return false;
//    }
//}

//bool JacoCustom::is_same_pose(jaco_msgs::FingerPosition* pose1, jaco_msgs::FingerPositionConstPtr pose2){
//    bool cond1 = pose1->finger1 == pose2->finger1;
//    bool cond2 = pose1->finger2 == pose2->finger2;
//    bool cond3 = pose1->finger3 == pose2->finger3;

//    if(cond1 && cond2 && cond3){
//        return true;
//    }

//    else{
//        return false;
//    }
//}


bool JacoCustom::is_same_pose_arm(sensor_msgs::JointState pose1, sensor_msgs::JointState pose2){
    bool cond1 = pose1.position.at(0) == pose1.position.at(0);
    bool cond2 = pose1.position.at(1) == pose1.position.at(1);
    bool cond3 = pose1.position.at(2) == pose1.position.at(2);
    bool cond4 = pose1.position.at(3) == pose1.position.at(3);
    bool cond5 = pose1.position.at(4) == pose1.position.at(4);
    bool cond6 = pose1.position.at(5) == pose1.position.at(5);


    if(cond1 && cond2 && cond3 && cond4 && cond5 && cond6){
        return true;
    }

    else{
        return false;
    }
}

bool JacoCustom::is_same_pose_fingers(sensor_msgs::JointState pose1, sensor_msgs::JointState pose2){
    bool cond1 = pose1.position.at(6) == pose2.position.at(6);
    bool cond2 = pose1.position.at(7) == pose2.position.at(7);
    bool cond3 = pose1.position.at(8) == pose2.position.at(8);

    if(cond1 && cond2 && cond3){
        return true;
    }

    else{
        return false;
    }
}



void JacoCustom::wait_for_arm_stopped(){
    arm_is_stopped_counter = 0;
    arm_is_stopped = false;
    check_arm_status = true;
    ros::Rate r(30);
    while(true){
        if(arm_is_stopped) break;
        else {r.sleep();}
    }
    std::cout << "Finished moving the arm!" << std::endl;
    check_arm_status = false;
}

void JacoCustom::wait_for_fingers_stopped(){
    fingers_are_stopped_counter = 0;
    fingers_are_stopped = false;
    check_fingers_status = true;
    ros::Rate r(30);
    while(true){
        if(fingers_are_stopped) break;
        else {r.sleep();}
    }
    std::cout << "Finished moving the fingers!" << std::endl;
    check_fingers_status = false;
}


//A simple test function to test moveit.
//void JacoCustom::moveAlongAxis(std::string axis, double distance){
//    // actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
//    //action_client.waitForServer();

//    if(axis == "x" || axis == "y" || axis == "z"){
//        jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();
//        double offset_x = 0;
//        double offset_y = 0;
//        double offset_z = 0;
//        if(axis == "x") offset_x = distance;
//        else if (axis == "y") offset_y = distance;
//        else offset_z = distance;

//        tf::TransformListener listener;
//        tf::StampedTransform tf_listened;
//        listener.waitForTransform("root","jaco_link_hand",ros::Time(0),ros::Duration(3.0));
//        listener.lookupTransform("root","jaco_link_hand",ros::Time(0),tf_listened);

//        geometry_msgs::Transform g_tf;
//        tf::transformTFToMsg(tf_listened, g_tf);


//        arm_mutex.lock();
//        pose_goal.pose.pose.position.x = g_tf.translation.x + offset_x;
//        pose_goal.pose.pose.position.y = g_tf.translation.y + offset_y;
//        pose_goal.pose.pose.position.z = g_tf.translation.z + offset_z;
//        pose_goal.pose.pose.orientation = g_tf.rotation;
//        pose_goal.pose.header.frame_id = "/root";
//        arm_mutex.unlock();

//        std::cout << "x: " << pose_goal.pose.pose.position.x << std::endl;
//        std::cout << "y: " << pose_goal.pose.pose.position.y << std::endl;
//        std::cout << "z: " << pose_goal.pose.pose.position.z << std::endl;

//        moveitPlugin(pose_goal.pose);
//    }
//}

/*
  New function to communicate with move_group.
  It publishes the pose that we need to compute a path.
  The listener (moveit_jaco_listener) will call the move_group node and execute the move command.
  We need this publisher because the move command needs to be in a separate queue.
  */

bool JacoCustom::moveitPlugin(geometry_msgs::PoseStamped p_pose){
    moveitPublisher.publish(p_pose);
    waitForMovementFinished();
    return move_it_status;
}


bool JacoCustom::moveitPlugin(tf::StampedTransform tf_pose){

    geometry_msgs::Transform g_tf;
    tf::transformTFToMsg(tf_pose, g_tf);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = g_tf.translation.x;
    pose.pose.position.y = g_tf.translation.y;
    pose.pose.position.z = g_tf.translation.z;
    pose.pose.orientation = g_tf.rotation;
    pose.header.frame_id = "/root";


    moveitPublisher.publish(pose);

    waitForMovementFinished();

    return move_it_status;
}

void JacoCustom::waitForMovementFinished(){
    moveit_status_received = false;
    ros::Rate r(3);

    // Waits until a message is received on the moveit status topic
    while(!moveit_status_received){
        r.sleep();
    }

}
