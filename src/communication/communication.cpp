#include <communication.h>

Communication::Communication(ros::NodeHandle p_nh)
{
    m_pub_coordinate = p_nh.advertise<std_msgs::String>("/coordinate_sender",1);
    m_pub_grasp = p_nh.advertise<std_msgs::String> ("/grasp_command_sender",1);
    m_pub_train = p_nh.advertise<std_msgs::String> ("/train_command_sender",1);
}

//-----------------------------------------------------------------------------------//
void Communication::callback_android_listener(const std_msgs::String &p_input)
{
    std::string temp_string_input = p_input.data;

    switch(temp_string_input[0])
    {
        case('c'):coordinate_processing(p_input);break;
        case('t'):train_processing(p_input);break;
        case('g'):grasp_processing(p_input);break;
    }
}

//---------------------------------------------------------------------------------//
void Communication::coordinate_processing(std_msgs::String p_coordinate)
{
    p_coordinate.data = p_coordinate.data.substr(2,p_coordinate.data.length());

    m_pub_coordinate.publish(p_coordinate);
}

//----------------------------------------------------------------------------------------//
void Communication::grasp_processing(std_msgs::String p_grasp)
{

}

//------------------------------------------------------------------------------------------//
void Communication::train_processing(std_msgs::String p_train)
{

}
