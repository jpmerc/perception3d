#include <ros/ros.h>
// PCL specific includes

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jaco_msgs/SetFingersPositionAction.h>
#include <jaco_msgs/ArmPoseAction.h>
#include <jaco_msgs/FingerPosition.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

geometry_msgs::PoseStamped arm_pose;
jaco_msgs::FingerPosition fingers_pose;

boost::mutex arm_mutex;
boost::mutex fingers_mutex;

bool end_program = false;

bool arm_is_stopped = false;
bool check_arm_status = false;
int arm_is_stopped_counter = 0;

bool fingers_are_stopped = false;
bool check_fingers_status = false;
int fingers_are_stopped_counter = 0;

void open_fingers();
void close_fingers();
void move_up(double distance);
void move_to_grasp_point(double x, double y, double z, double rotx, double roty, double rotz, double rotw);
bool is_same_pose(geometry_msgs::PoseStamped* pose1, const geometry_msgs::PoseStampedConstPtr pose2);
bool is_same_pose(jaco_msgs::FingerPosition* pose1, jaco_msgs::FingerPositionConstPtr pose2);
void wait_for_arm_stopped();
void wait_for_fingers_stopped();

void arm_position_callback (const geometry_msgs::PoseStampedConstPtr& input_pose){

    if(check_arm_status){
        bool same_pose = is_same_pose(&arm_pose,input_pose);
        if(same_pose){
            arm_is_stopped_counter++;
        }
        if(arm_is_stopped_counter >= 5){
            arm_is_stopped = true;
        }
    }

    // Assign new value to arm pose
    arm_mutex.lock();
    arm_pose = *input_pose;
    arm_mutex.unlock();
}

void fingers_position_callback(const jaco_msgs::FingerPositionConstPtr& input_fingers){
    if(check_fingers_status){
        bool same_pose = is_same_pose(&fingers_pose,input_fingers); //create a new function
        if(same_pose){
            fingers_are_stopped_counter++;
        }
        if(fingers_are_stopped_counter >= 5){
            fingers_are_stopped = true;
        }
    }

    // Assign new value to arm pose
    fingers_mutex.lock();
    fingers_pose = *input_fingers;
    fingers_mutex.unlock();
}

void thread_function(){
    if(ros::ok()){
        open_fingers();
        move_to_grasp_point(-0.24, 0.366, -0.003, 0.064, -0.658, -0.035, 0.75);
        close_fingers();
        move_up(0.4);
        end_program = true;
    }
}

bool is_same_pose(geometry_msgs::PoseStamped* pose1, geometry_msgs::PoseStampedConstPtr pose2){
    bool cond1 = pose1->pose.position.x == pose2->pose.position.x;
    bool cond2 = pose1->pose.position.y == pose2->pose.position.y;
    bool cond3 = pose1->pose.position.z == pose2->pose.position.z;
    bool cond4 = pose1->pose.orientation.x == pose2->pose.orientation.x;
    bool cond5 = pose1->pose.orientation.y == pose2->pose.orientation.y;
    bool cond6 = pose1->pose.orientation.z == pose2->pose.orientation.z;
    bool cond7 = pose1->pose.orientation.w == pose2->pose.orientation.w;

    if(cond1 && cond2 && cond3 && cond4 && cond5 && cond6 && cond7){
        return true;
    }

    else{
        return false;
    }
}

bool is_same_pose(jaco_msgs::FingerPosition* pose1, jaco_msgs::FingerPositionConstPtr pose2){
    bool cond1 = pose1->Finger_1 == pose2->Finger_1;
    bool cond2 = pose1->Finger_2 == pose2->Finger_2;
    bool cond3 = pose1->Finger_3 == pose2->Finger_3;

    if(cond1 && cond2 && cond3){
        return true;
    }

    else{
        return false;
    }
}

void wait_for_arm_stopped(){
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

void wait_for_fingers_stopped(){
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

void move_to_grasp_point(double x, double y, double z, double rotx, double roty, double rotz, double rotw){
    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
    action_client.waitForServer();
    jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();

    pose_goal.pose.header.frame_id = "/jaco_api_origin";
    pose_goal.pose.pose.position.x = x;
    pose_goal.pose.pose.position.y = y;
    pose_goal.pose.pose.position.z = z;
    pose_goal.pose.pose.orientation.x = rotx;
    pose_goal.pose.pose.orientation.y = roty;
    pose_goal.pose.pose.orientation.z = rotz;
    pose_goal.pose.pose.orientation.w = rotw;
    action_client.sendGoal(pose_goal);

    wait_for_arm_stopped();
}


void open_fingers(){
    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> action_client("jaco/finger_joint_angles",true);
    action_client.waitForServer();
    jaco_msgs::SetFingersPositionGoal fingers = jaco_msgs::SetFingersPositionGoal();
    fingers.fingers.Finger_1 = 0;
    fingers.fingers.Finger_2 = 0;
    fingers.fingers.Finger_3 = 0;
    action_client.sendGoal(fingers);
    wait_for_fingers_stopped();
    std::cout << "Grasp completed" << std::endl;
}


void close_fingers(){
    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> action_client("jaco/finger_joint_angles",true);
    action_client.waitForServer();
    jaco_msgs::SetFingersPositionGoal fingers = jaco_msgs::SetFingersPositionGoal();
    fingers.fingers.Finger_1 = 60;
    fingers.fingers.Finger_2 = 60;
    fingers.fingers.Finger_3 = 60;
    action_client.sendGoal(fingers);
    wait_for_fingers_stopped();
    std::cout << "Grasp completed" << std::endl;
}


void move_up(double distance){
    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
    action_client.waitForServer();
    jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();

    arm_mutex.lock();
    pose_goal.pose = arm_pose;
    pose_goal.pose.header.frame_id = "/jaco_api_origin";
    pose_goal.pose.pose.position.z += distance;
    arm_mutex.unlock();

    action_client.sendGoal(pose_goal);
    wait_for_arm_stopped();
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "grasp");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Subscriber sub = n.subscribe ("/jaco/tool_position", 1, arm_position_callback);
    ros::Subscriber sub2 = n.subscribe ("/jaco/finger_position", 1, fingers_position_callback);
    boost::thread thread_(thread_function);

    ros::Rate r(30);
    while(ros::ok() && !end_program){
        ros::spinOnce();
        r.sleep();
    }
    thread_.join();

    return 0;
}
