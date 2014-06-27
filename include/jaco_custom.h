#ifndef jaco_custom_H
#define jaco_custom_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jaco_msgs/SetFingersPositionAction.h>
#include <jaco_msgs/ArmPoseAction.h>
#include <jaco_msgs/FingerPosition.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>


class JacoCustom{
public:
    JacoCustom(ros::NodeHandle &node);
    void arm_position_callback (const geometry_msgs::PoseStampedConstPtr& input_pose);
    void fingers_position_callback(const jaco_msgs::FingerPositionConstPtr& input_fingers);
    void open_fingers();
    void close_fingers();
    void move_up(double distance);
    void moveToPoint(double x, double y, double z, double rotx, double roty, double rotz, double rotw);
    geometry_msgs::PoseStamped getArmPosition();
    jaco_msgs::FingerPosition getFingersPosition();

    geometry_msgs::PoseStamped getGraspArmPosition();

private:

    bool is_same_pose(geometry_msgs::PoseStamped* pose1, const geometry_msgs::PoseStampedConstPtr pose2);
    bool is_same_pose(jaco_msgs::FingerPosition* pose1, jaco_msgs::FingerPositionConstPtr pose2);

    void wait_for_arm_stopped();
    void wait_for_fingers_stopped();

    geometry_msgs::PoseStamped arm_pose;
    jaco_msgs::FingerPosition fingers_pose;
    boost::mutex arm_mutex;
    boost::mutex fingers_mutex;

    bool end_program;

    bool arm_is_stopped;
    bool check_arm_status;
    int arm_is_stopped_counter;

    bool fingers_are_stopped;
    bool check_fingers_status;
    int fingers_are_stopped_counter;


};

#endif
