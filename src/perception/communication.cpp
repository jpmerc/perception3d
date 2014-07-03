#include <communication.h>

Communication::Communication(ObjectExtractor *p_obj_e, fileAPI *p_api, JacoCustom *p_jaco)
{
    m_object_ex_ptr = p_obj_e;
    m_api_ptr = p_api;
    m_jaco_ptr = p_jaco;
    m_coordinate_received = false;
    m_grasp_received = false;
    m_train_received = false;
}

//-----------------------------------------------------------------------------------//
void Communication::callback_android_listener(const std_msgs::String &p_input)
{

    switch(p_input.data[0])
    {
    case('c'):coordinate_processing(p_input);break;
    case('t'):train_processing(p_input);break;
    case('g'):grasp_processing(p_input);break;
    default:break;
    }
}

//---------------------------------------------------------------------------------//
void Communication::coordinate_processing(std_msgs::String p_coordinate)
{
    m_coordinate_received = true;
    m_grasp_received = false;
    m_train_received = false;

    std::string string_temp = "";
    for(int i = 0; i < p_coordinate.data.size(); i ++)
    {
        if(p_coordinate.data.at(i) == '_')
        {
            m_coordinate_user_sended[0] = atof(string_temp.c_str());
            string_temp.clear();
        }
        else
            string_temp += p_coordinate.data.at(i);

    }
    m_coordinate_user_sended[1] = atof(string_temp.c_str());
    m_coordinate_user_sended[0] = m_coordinate_user_sended[0]*640;
    m_coordinate_user_sended[1] = m_coordinate_user_sended[1]*480;
}

//----------------------------------------------------------------------------------------//
void Communication::grasp_processing(std_msgs::String p_grasp)
{
    m_coordinate_received = false;
    m_grasp_received = false;
    m_train_received = true;
}

//------------------------------------------------------------------------------------------//
void Communication::train_processing(std_msgs::String p_train)
{
    m_coordinate_received = false;
    m_grasp_received = false;
    m_train_received = true;
}

//------------------------------------------------------------------------------------------//
bool Communication::get_coordinate_received() const
{
    return m_coordinate_received;
}

//--------------------------------------------------------------------------------------------//
bool Communication::get_grasp_received() const
{
    return m_grasp_received;
}

//---------------------------------------------------------------------------------------------//
bool Communication::get_train_received() const
{
    return m_train_received;
}

//-----------------------------------------------------------------------------------------------//
void Communication::spin_once()
{
    if(m_coordinate_received)
    {
        m_object_ex_ptr->coordinate_processing(m_coordinate_user_sended);
        m_object_ex_ptr->spin_once();
    }
    else if(m_train_received)
    {
        train();
    }
    else if(m_grasp_received)
    {

    }
    else
    {
        m_object_ex_ptr->spin_once();
    }
    m_coordinate_received = false;
    m_grasp_received = false;
    m_train_received = false;
}

//-----------------------------------------------------------------------------------------------//
void Communication::train(){
    //
    Object obj;
    //obj.name = m_api_ptr->findDefaultName();
    //obj.object_pointcloud = m_object_ex_ptr->getObjectToGrasp();

    //obj.object_pose = ; // find object pose

    tf::StampedTransform arm_pose_before_grasp = m_jaco_ptr->getGraspArmPosition();

    // For testing purposes only, comment the following line and uncomment previous one
   // tf::StampedTransform arm_pose_before_grasp = m_jaco_ptr->getArmPositionFromCamera();


    cout << "arm pose : [" <<   arm_pose_before_grasp.getOrigin().getX() << ", " <<
            arm_pose_before_grasp.getOrigin().getY() << ", " <<
            arm_pose_before_grasp.getOrigin().getZ() << "]" << endl;


    // find the arm pose in the camera frame and then compute the difference between the 2 poses
    Eigen::Vector4f object_pose = m_object_ex_ptr->getGraspCentroid();
    cout << "object pose : [" <<  object_pose[2] << ", " << -object_pose[0] << ", " << -object_pose[1] << "]" << endl;

    tf::Pose tf_pose;
    tf_pose.setIdentity();
    tf_pose.setOrigin(tf::Vector3(object_pose[2],-object_pose[0],-object_pose[1]));
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "camera_rgb_frame", "detected_object_centroids"));

}





































