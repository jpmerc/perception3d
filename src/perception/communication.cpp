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
/*
  The call back when the apps send a coordinate.
  All the message send begin with a letter (ex c_...).  The callBack will execute the right command.
  */
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
/*
  Parse the the message received from the tablet when its a coordinate.
  Param[in]  std_msgs::String a string message that tha tablet send.
  */
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
/*
  Parse the the message received from the tablet when the user deceide to take the object.
  Param[in]  std_msgs::String a string message that tha tablet send.
  */
void Communication::grasp_processing(std_msgs::String p_grasp)
{
    m_coordinate_received = false;
    m_grasp_received = false;
    m_train_received = true;
}

//------------------------------------------------------------------------------------------//
/*
  Parse the the message received from the tablet when the user deceide to train.
  Param[in]  std_msgs::String a string message that tha tablet send.
  */
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
        // Training and repeating steps should be in 2 steps :
        // 1) Check if object is recognized and load all appropriate data in Graphical User Interface
        // 2) Then Call the function train/repeat (depending on which button is clicked) with arguments (recognized or not, which pose is selected to grasp, etc.)

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


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pointcloud = m_object_ex_ptr->getObjectToGrasp();
    Eigen::Matrix4f calculated_object_transform;
    int object_index = m_object_ex_ptr->m_object_recognition.OURCVFHRecognition(input_pointcloud, m_api_ptr, calculated_object_transform);

    bool known_object = true;
    if(object_index < 0){
        known_object = false;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature;
    std::vector<tf::Transform> object_pose_vector;
    std::vector<tf::Transform> relative_arm_pose_vector;
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > surface_transforms;
    std::string name;

    ObjectBd obj;

    if(known_object){
        //Load object from Histogram position
        obj = m_api_ptr->retrieveObjectFromHistogram(object_index);
        object_pointcloud = m_object_ex_ptr->m_object_recognition.transformAndVoxelizePointCloud(input_pointcloud, obj.getPointCloud(),calculated_object_transform);
        object_pose_vector = obj.getObjectPose();
        relative_arm_pose_vector = obj.getArmPose();

        surface_transforms = obj.getTransforms();
        name = obj.getName();
    }

    else{
        object_pointcloud = input_pointcloud;
    }

    // Calculate surface signatures and transforms (coordinate systems)
    object_signature = m_object_ex_ptr->m_object_recognition.makeCVFH(object_pointcloud,surface_transforms);


    // **************  OBJECT POSE (Change later for position of the object in the map)**************************************************************************
    tf::StampedTransform object_tf = m_object_ex_ptr->getCentroidPositionRGBFrame();
    object_pose_vector.push_back(object_tf);

    // JACO POSE
    tf::StampedTransform arm_pose_before_grasp = m_jaco_ptr->getGraspArmPosition();
    // For testing purposes only, comment the following line and uncomment previous one
    //tf::StampedTransform arm_pose_before_grasp = m_jaco_ptr->getArmPositionFromCamera();

    // RELATIVE POSE
    tf::Transform arm_rel_pose;
    tf::Vector3 translation = arm_pose_before_grasp.getOrigin() - object_tf.getOrigin();
    arm_rel_pose = tf::Transform(object_tf.getBasis().transposeTimes(arm_pose_before_grasp.getBasis()), translation);
    relative_arm_pose_vector.push_back(arm_rel_pose);

    // TO VIEW FRAMES
    //static tf::TransformBroadcaster br;
    //br.sendTransform(object_tf);
    //br.sendTransform(tf::StampedTransform(diff,ros::Time::now(),"detected_object_centroids","jaco_relative_pose"));

    // PRINT POSE
    //    cout << "arm pose : [" <<   arm_pose_before_grasp.getOrigin().getX() << ", " <<
    //            arm_pose_before_grasp.getOrigin().getY() << ", " <<
    //            arm_pose_before_grasp.getOrigin().getZ() << "]" << endl;


    // SAVE
    if(known_object){
        obj.setAllAttribut(name,object_signature,object_pointcloud,relative_arm_pose_vector,object_pose_vector,surface_transforms);
    }

    else{
        m_api_ptr->save(object_signature,object_pointcloud,relative_arm_pose_vector,object_pose_vector,surface_transforms);
    }

    m_relative_pose = arm_rel_pose;


}

void Communication::repeat(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pointcloud = m_object_ex_ptr->getObjectToGrasp();
    Eigen::Matrix4f calculated_object_transform;
    int object_index = m_object_ex_ptr->m_object_recognition.OURCVFHRecognition(input_pointcloud, m_api_ptr, calculated_object_transform);

    // Object is recognized
    if(object_index >= 0){
       ObjectBd obj = m_api_ptr->retrieveObjectFromHistogram(object_index);

       // Choose good index of object_tf

       // Select good index of arm_relative_pose (grasp position)

       // Publish TF in a thread and listen to it

       // Move the arm (MoveIt instead of direct moveToPoint function)

    }



    /*
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

    m_jaco_ptr->moveToPoint(goal_pose);*/


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


void Communication::testTFandSurfaceTransforms(){


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pointcloud = m_object_ex_ptr->getObjectToGrasp();
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > tf_vector;
    std::vector<Eigen::Vector3f> centroidVec;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr sig = m_object_ex_ptr->m_object_recognition.makeCVFH(input_pointcloud,tf_vector, centroidVec);


    static tf::TransformBroadcaster br;
    tf::StampedTransform object_tf = m_object_ex_ptr->getCentroidPositionRGBFrame();
    br.sendTransform(object_tf);

    /*
      To get the transfrom work we need to get the centroid directly from the ourcvfh algo.
      I make a new function that take the centroid in parameters.
      So this way we can set the orrigin of the tf
      */
    m_object_ex_ptr->m_transform_pc->clear();
    for(int i=0; i < tf_vector.size(); i++){
        Eigen::Matrix4f matrix = tf_vector.at(i);
        tf::Transform tf_ = tfFromEigen(matrix);
        tf::Vector3 vec = tf_.getOrigin();
        tf::Vector3 vec2(centroidVec[i](2,0), -(centroidVec[i](0,0)), -(centroidVec[i](1,0)));
        //tf_.setOrigin(tf::Vector3(vec.getZ(),-vec.getX(),-vec.getY()));
        tf_.setOrigin(vec2);
        std::stringstream ss;
        ss << i;
        std::string str = "surfaceTransform_" + ss.str();
        br.sendTransform(tf::StampedTransform(tf_,ros::Time::now(),"camera_rgb_frame",str));

        PointT pt;
        pt.x = vec.getX(); pt.y = vec.getY(); pt.z = vec.getZ();
        m_object_ex_ptr->m_transform_pc->push_back(pt);

    }

}


tf::Transform Communication::tfFromEigen(Eigen::Matrix4f trans)
{
    Eigen::Matrix4f mf = trans; //The matrix I want to convert
    Eigen::Matrix4d md(mf.cast<double>());
    Eigen::Affine3d affine(md);
    tf::Transform transform_;
    tf::transformEigenToTF(affine,transform_);
    tf::Quaternion test = transform_.getRotation().normalize();
    transform_.setRotation(test);



    return transform_;
}





