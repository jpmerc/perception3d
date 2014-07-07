#include <ros/ros.h>
// PCL specific includes

#include <jaco_msgs/JointAngles.h>
#include <angles/angles.h>
#include <sensor_msgs/JointState.h>

ros::Publisher pub;
using namespace std;

int seq = 0;

void angles_callback(const jaco_msgs::JointAnglesConstPtr& input_fingers){
    seq++;

    const char* nameArgs[] = {"jaco_joint_1", "jaco_joint_2", "jaco_joint_3", "jaco_joint_4", "jaco_joint_5", "jaco_joint_6"};
    std::vector<std::string> JointName(nameArgs, nameArgs+6);

    sensor_msgs::JointState joint_state;
    joint_state.header.seq = seq;
    joint_state.name = JointName;

    joint_state.position.resize(6);
    joint_state.velocity.resize(6);
    joint_state.effort.resize(6);

    joint_state.position[0] = angles::normalize_angle(angles::from_degrees(input_fingers->Angle_J1));
    joint_state.position[1] = angles::normalize_angle(angles::from_degrees(input_fingers->Angle_J2));
    joint_state.position[2] = angles::normalize_angle(angles::from_degrees(input_fingers->Angle_J3));
    joint_state.position[3] = angles::normalize_angle(angles::from_degrees(input_fingers->Angle_J4));
    joint_state.position[4] = angles::normalize_angle(angles::from_degrees(input_fingers->Angle_J5));
    joint_state.position[5] = angles::normalize_angle(angles::from_degrees(input_fingers->Angle_J6));


    pub.publish(joint_state);
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "grasp");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Subscriber sub = n.subscribe ("/jaco/joint_angles", 1, angles_callback);
    pub = n.advertise<sensor_msgs::JointState> ("/custom_jaco/joint_states",1);

    ros::spin();


    return 0;
}
