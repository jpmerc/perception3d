#include <ros/ros.h>
#include <std_msgs/String.h>

class Communication
{
public:

    Communication(ros::NodeHandle p_nh);
    void callback_android_listener(const std_msgs::String& p_input);
    void coordinate_processing(std_msgs::String p_coordinate);
    void grasp_processing(std_msgs::String p_grasp);
    void train_processing(std_msgs::String p_train);


private:

    ros::Publisher m_pub_coordinate;
    ros::Publisher m_pub_grasp;
    ros::Publisher m_pub_train;

    std_msgs::String m_coord_send;
    std_msgs::String m_grasp_send;
    std_msgs::String m_train_send;

};
