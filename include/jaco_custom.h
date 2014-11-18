#ifndef JACO_CUSTOM_H
#define JACO_CUSTOM_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <jaco_msgs/SetFingersPositionAction.h>
//#include <jaco_msgs/ArmPoseAction.h>
//#include <jaco_msgs/FingerPosition.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <sensor_msgs/JointState.h>
#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/EulerToQuaternion.h>
#include <wpi_jaco_msgs/ExecuteGraspGoal.h>
#include <wpi_jaco_msgs/ExecuteGraspAction.h>
#include <std_msgs/Bool.h>


class JacoCustom{
public:
    JacoCustom(ros::NodeHandle &node);
//    void arm_position_callback (const geometry_msgs::PoseStampedConstPtr& input_pose);
//    void fingers_position_callback(const jaco_msgs::FingerPositionConstPtr& input_fingers);

    void joint_state_callback (const sensor_msgs::JointStateConstPtr& input_pose);
    void moveit_move_status_callback (const std_msgs::BoolConstPtr in_status);
    void open_fingers();
    void close_fingers();
    void move_up(double distance);
    void moveToPoint(double x, double y, double z, double rotx, double roty, double rotz, double rotw);
    void moveToPoint(tf::Transform tf_);


    geometry_msgs::PoseStamped getArmPosition();
//    jaco_msgs::FingerPosition getFingersPosition();
    tf::StampedTransform getArmPositionFromCamera();
    tf::StampedTransform getGraspArmPosition();

    std::vector<double> getFingersPosition();

    void moveAlongAxis(std::string axis, double distance);
    bool moveitPlugin(geometry_msgs::PoseStamped p_pose);//The communication chanel to moveit
    bool moveitPlugin(tf::StampedTransform tf_pose);

private:

//    bool is_same_pose(geometry_msgs::PoseStamped* pose1, const geometry_msgs::PoseStampedConstPtr pose2);
//    bool is_same_pose(jaco_msgs::FingerPosition* pose1, jaco_msgs::FingerPositionConstPtr pose2);
    bool is_same_pose_arm(sensor_msgs::JointState pose1, sensor_msgs::JointState pose2);
    bool is_same_pose_fingers(sensor_msgs::JointState pose1, sensor_msgs::JointState pose2);


    void wait_for_arm_stopped();
    void wait_for_fingers_stopped();
    void waitForMovementFinished();

//    geometry_msgs::PoseStamped arm_pose;
//    jaco_msgs::FingerPosition fingers_pose;
    sensor_msgs::JointStateConstPtr joint_state;

    boost::mutex arm_mutex;
    boost::mutex fingers_mutex;
    boost::mutex joint_mutex;

    bool end_program;

    bool arm_is_stopped;
    bool check_arm_status;
    int arm_is_stopped_counter;

    bool fingers_are_stopped;
    bool check_fingers_status;
    int fingers_are_stopped_counter;

    tf::Transform tool_position_tf;
    tf::TransformBroadcaster position_broadcaster;

    ros::Publisher moveitPublisher;//New publisher, from now we absolutely need the nodehandle to run this class
    ros::Publisher angular_publisher;
    ros::Publisher cartesian_publisher;

    ros::ServiceClient cartesian_position_service_client;

    bool moveit_status_received;
    bool move_it_status;

};

#endif
