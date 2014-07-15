#include <communication.h>

Communication::Communication(ObjectExtractor *p_obj_e, FileAPI *p_api, JacoCustom *p_jaco)
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
        m_position_vector_cvfh = m_object_ex_ptr->coordinate_processing(m_coordinate_user_sended,
                                                                        m_api_ptr->getAllHistograms());

        m_object_ex_ptr->coordinate_processing(m_coordinate_user_sended,m_api_ptr->getAllHistograms());
        m_object_ex_ptr->spin_once();

        //pour retrouver l'object qui est reconnue par le cvfh
        //m_api_ptr->retrieveObjectFromHistogramme(m_position_vector_cvfh);
    }
    else if(m_train_received)
    {
        train();
    }
    else if(m_grasp_received)
    {
        repeat();
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud = m_object_ex_ptr->getObjectToGrasp();
    int positionVectorObject = m_object_ex_ptr->m_object_recognition.object_recon(object_pointcloud, m_api_ptr->getAllHistograms());

    bool known_object = true;
    if(positionVectorObject <= -1){
        known_object = false;
    }

    if(known_object){
        //Load object from Histogram position
        ObjectBd obj = m_api_ptr->retrieveObjectFromHistogram(positionVectorObject);

        // Find transformation between scans (new one on old one, so that relative arm position is kept) and merge them
        pcl::PointCloud<PointT>::Ptr sampled_model_pc;
        pcl::PointCloud<PointT>::Ptr sampled_object_pc;
        m_object_ex_ptr->m_object_recognition.computeUniformSampling(obj.getPointCloud(),sampled_model_pc);
        m_object_ex_ptr->m_object_recognition.computeUniformSampling(object_pointcloud,sampled_object_pc);


        pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature = m_object_ex_ptr->m_object_recognition.makeCVFH(object_pointcloud);
    //    m_object_ex_ptr->m_object_recognition.mergePointCVFH();

        //Eigen::Matrix4f mf = mergeScans(obj.getPointCloud(),object_pointcloud);
        //Eigen::Matrix4d md(object_orientation.cast());
        //Eigen::Affine3d affine(md);
        // tf::Transform object_transform;
        // tf::TransformEigenToTF(affine, object_transform);
        // obj.pointcloud = new_pc;

        // Calculate CVFH signature on new pointcloud
        //obj.updateSignature();

        //Push new pose in pose vector (bug to correct : overwrite file instead of appending in fileAPI)

        // JACO POSE
        //tf::StampedTransform arm_pose_before_grasp = m_jaco_ptr->getGraspArmPosition();
        // For testing purposes only, comment the following line and uncomment previous one
        tf::StampedTransform arm_pose_before_grasp = m_jaco_ptr->getArmPositionFromCamera();

        // Pose relative to pointcloud model orientation
        //tf::Vector3 translation = arm_pose_before_grasp.getOrigin() + object_transform.getOrigin();
        //tf::Transform new_relative_pose = tf::Transform(arm_pose_before_grasp.getRotation()*object_transform.getRotation(), translation);
        //obj.getArmPose().push_back(new_relative_pose);


        // Object Pose in the map
        //Position new_object_position = findObjectPositionInMap();
        //obj.getObjectPose().push_back(new_object_position);

        //save object
        m_api_ptr->saveObject(obj);

    }

    else{
        //
        //Object obj;
        //obj.name = m_api_ptr->findDefaultName();

        pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature = m_object_ex_ptr->m_object_recognition.makeCVFH(object_pointcloud);

        // OBJECT POSE
        //obj.object_pose = ; // find object pose
        tf::StampedTransform object_tf = m_object_ex_ptr->getCentroidPositionRGBFrame();

        // JACO POSE
        //tf::StampedTransform arm_pose_before_grasp = m_jaco_ptr->getGraspArmPosition();
        // For testing purposes only, comment the following line and uncomment previous one
        tf::StampedTransform arm_pose_before_grasp = m_jaco_ptr->getArmPositionFromCamera();

        // RELATIVE POSE
        tf::Transform arm_rel_pose;
        tf::Vector3 translation = arm_pose_before_grasp.getOrigin() - object_tf.getOrigin();
        arm_rel_pose = tf::Transform(object_tf.getBasis().transposeTimes(arm_pose_before_grasp.getBasis()), translation);

        // TO VIEW FRAMES
        //static tf::TransformBroadcaster br;
        //br.sendTransform(object_tf);
        //br.sendTransform(tf::StampedTransform(diff,ros::Time::now(),"detected_object_centroids","jaco_relative_pose"));

        // PRINT POSE
        //    cout << "arm pose : [" <<   arm_pose_before_grasp.getOrigin().getX() << ", " <<
        //            arm_pose_before_grasp.getOrigin().getY() << ", " <<
        //            arm_pose_before_grasp.getOrigin().getZ() << "]" << endl;


        // SAVE
        //m_api_ptr->save(object_signature,object_pointcloud,arm_rel_pose,object_tf);
        m_relative_pose = arm_rel_pose;

    }
}

void Communication::repeat(){
    tf::StampedTransform object_tf = m_object_ex_ptr->getCentroidPositionRGBFrame();
    tf::Vector3 translation2 = object_tf.getOrigin() + m_relative_pose.getOrigin();
    tf::Transform tf_ = tf::Transform(object_tf.getRotation()*m_relative_pose.getRotation(), translation2);

    m_publish_relative_pose = true;
    boost::thread thread(&Communication::publishRelativePoseTF,this,tf_);
    tf::StampedTransform goal_pose;
    tf::TransformListener listener;
    listener.waitForTransform("jaco_api_origin","jaco_tool_relative_pose",ros::Time(0),ros::Duration(3.0));
    listener.lookupTransform("jaco_api_origin","jaco_tool_relative_pose",ros::Time(0),goal_pose);
    m_publish_relative_pose = false;
    thread.join();

    m_jaco_ptr->moveToPoint(goal_pose);


}

void Communication::publishRelativePoseTF(tf::Transform relative_pose){
    static tf::TransformBroadcaster br;
    ros::Rate r(10);
    while(m_publish_relative_pose){
        tf::StampedTransform arm_relative = tf::StampedTransform(relative_pose,ros::Time::now(),"camera_rgb_frame","jaco_tool_relative_pose");
        br.sendTransform(arm_relative);
        r.sleep();
    }

}





























