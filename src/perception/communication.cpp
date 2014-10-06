#include <communication.h>
using namespace std;

Communication::Communication(ObjectExtractor *p_obj_e, FileAPI *p_api, JacoCustom *p_jaco)
{
    m_object_ex_ptr = p_obj_e;
    m_api_ptr = p_api;
    m_jaco_ptr = p_jaco;
    m_coordinate_received = false;
    m_grasp_received = false;
    m_train_received = false;
    selected_object_index = -1;
    grasp_list_index = -1;

    saveToDBWithoutArmPoseThread = boost::thread(&Communication::saveToDBWithoutArmPose,this);
    transforms_vector.clear();
//    recognitionViewer.reset(new pcl::visualization::PCLVisualizer("Recognition Tests"));
//    recognitionViewer->setBackgroundColor (0, 0, 0);
//    recognitionViewer->initCameraParameters ();
//    recognitionViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
//    vtkSmartPointer<vtkRenderWindow> renderWindow = recognitionViewer->getRenderWindow();
//    renderWindow->SetSize(800,450);
//    renderWindow->Render();
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

    string str = p_grasp.data;
    int str_size = str.size();
    string str_grasp_index = str.substr(2,str_size-1);
    grasp_list_index = atoi(str_grasp_index.c_str());

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
        selected_pointcloud = m_object_ex_ptr->coordinate_processing(m_coordinate_user_sended);
        m_object_ex_ptr->setObjectToGrasp(selected_pointcloud);

        if(selected_pointcloud->size() > 0){
            // An object was clicked on
            selected_object_index = m_object_ex_ptr->m_object_recognition.OURCVFHRecognition(selected_pointcloud, m_api_ptr, calculated_object_transform, transforms_vector);

            // Refresh image and pointcloud
            m_object_ex_ptr->spin_once();

            // Fill user interface with object info
            fillUserInterfaceWithObjectInfo();

        }

        else{
            //There is no object where the user clicked
            m_object_ex_ptr->publishToAndroidDevice("no_object");
        }



    }
    else if(m_train_received)
    {
        // Training and repeating steps should be in 2 steps :
        // 1) Check if object is recognized and load all appropriate data in Graphical User Interface
        // 2) Then Call the function train/repeat (depending on which button is clicked) with arguments (recognized or not, which pose is selected to grasp, etc.)

        train(true, false);
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
void Communication::train(bool saveJacoPose, bool viewTF){

    pcl::PointCloud<PointT>::Ptr scan_pc = m_object_ex_ptr->getObjectToGrasp();
    scan_pc = m_object_ex_ptr->m_object_recognition.smoothSurfaces(scan_pc);

    selected_object_index = m_object_ex_ptr->m_object_recognition.OURCVFHRecognition(scan_pc, m_api_ptr, calculated_object_transform, transforms_vector);

    bool known_object = true;
    if(selected_object_index < 0){
        known_object = false;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature(new pcl::PointCloud<pcl::VFHSignature308>());
    std::vector<tf::Transform> object_pose_vector;
    std::vector<tf::Transform> relative_arm_pose_vector;
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > surface_transforms;
    std::string name;

    ObjectBd obj;


    if(known_object){
        //Load object from Histogram position
        obj = m_api_ptr->retrieveObjectFromHistogram(selected_object_index);
        object_pointcloud = m_object_ex_ptr->m_object_recognition.transformAndVoxelizePointCloud(scan_pc, obj.getPointCloud(),calculated_object_transform.inverse());
        object_pose_vector = obj.getObjectPose();
        relative_arm_pose_vector = obj.getArmPose();
        //surface_transforms = obj.getTransforms(); //not needed, transforms will be calculated on the new and merged pointcloud
        name = obj.getName();
    }

    else{
        object_pointcloud = scan_pc;

        // **************  OBJECT POSE (Change later for position of the object in the map)  ********************** camera_rgb_frame -> detected_object_centroids
        tf::StampedTransform object_tf = m_object_ex_ptr->getCentroidPositionRGBFrame();
        object_pose_vector.push_back(object_tf);

    }

    if(object_pointcloud && object_pointcloud->size() > 0){

        // Calculate surface signatures and transforms (coordinate systems)
        object_signature = m_object_ex_ptr->m_object_recognition.makeCVFH(object_pointcloud,surface_transforms);


        // Initialize the transforms in case the
        tf::StampedTransform arm_pose_before_grasp;
        arm_pose_before_grasp.setIdentity();
        tf::Transform arm_rel_pose;
        arm_rel_pose.setIdentity();

        if(saveJacoPose){
            // JACO POSE (camera_rgb_frame -> jaco_tool_position)
            std::cout << "Teach the grasp to the system!" << std::endl;
            arm_pose_before_grasp = m_jaco_ptr->getGraspArmPosition();
            // For testing purposes only, comment the following line and uncomment previous one
            //arm_pose_before_grasp = m_jaco_ptr->getArmPositionFromCamera();

            // SAVE ARM POSE (SCAN OBJECT) IN KINECT COORDINATE SYSTEM
            arm_rel_pose =  m_object_ex_ptr->m_object_recognition.transformWorldFrameToKinectFrame(arm_pose_before_grasp);

            if(known_object){
                // Align arm pose to database reference frame (inverse transform since tf is from db to scan)
                arm_rel_pose =  transforms_vector[2].inverse() * arm_rel_pose;
            }

        }
        relative_arm_pose_vector.push_back(arm_rel_pose);

        // TO VIEW FRAMES
        if(viewTF){
//            static tf::TransformBroadcaster br;
//            br.sendTransform(object_tf);
//            br.sendTransform(arm_pose_before_grasp);
//            br.sendTransform(tf::StampedTransform(arm_rel_pose,ros::Time::now(),"detected_object_centroids","jaco_tool_position_over"));
        }

        // SAVE
        if(known_object){
            obj.setAllAttribut(name,object_signature,object_pointcloud,relative_arm_pose_vector,object_pose_vector,surface_transforms);
            m_api_ptr->saveObject(obj);
        }

        else{
            m_api_ptr->save(object_signature,object_pointcloud,relative_arm_pose_vector,object_pose_vector,surface_transforms);
        }

        m_relative_pose = arm_rel_pose;
    }
}

void Communication::repeat(){

    pcl::PointCloud<PointT>::Ptr scan_pc = m_object_ex_ptr->getObjectToGrasp();
    scan_pc = m_object_ex_ptr->m_object_recognition.smoothSurfaces(scan_pc);

    selected_object_index = m_object_ex_ptr->m_object_recognition.OURCVFHRecognition(scan_pc, m_api_ptr, calculated_object_transform, transforms_vector);


    ObjectBd obj = m_api_ptr->retrieveObjectFromHistogram(selected_object_index);

    // Select good index of arm_relative_pose (grasp position) from the list in User Interface
    int graspIndex = grasp_list_index;
    if(graspIndex < 0){
        graspIndex = 0;
    }

    // Arm position -> object centroid (in training)
    Eigen::Matrix4f arm_pose_kinect_matrix;
    pcl_ros::transformAsMatrix(obj.getArmPose().at(graspIndex),arm_pose_kinect_matrix);
    Eigen::Matrix4f transformation = calculated_object_transform * arm_pose_kinect_matrix;
    tf::Transform arm_pose_world_frame = m_object_ex_ptr->m_object_recognition.transformKinectFrameToWorldFrame(transformation);


    // Pose of database object in scene when it was trained (The reference is always the original pointcloud)
//    tf::Transform model_pose = obj.getObjectPose().at(0);
//    tf::Transform object_tf = tf::Transform(m_object_ex_ptr->getCentroidPositionRGBFrame());
//    tf::Transform environment_arm_pose;
//    environment_arm_pose.setOrigin(model_pose.getOrigin() + scene_to_model.getOrigin() + arm_pose_kinect_frame.getOrigin());
//    environment_arm_pose.setRotation(model_pose.getRotation()*scene_to_model.getRotation()*arm_pose_kinect_frame.getRotation());
//    geometry_msgs::Transform g_tf;
//    tf::transformTFToMsg(environment_arm_pose,g_tf);

//    tf::StampedTransform object_tf = m_object_ex_ptr->getCentroidPositionRGBFrame();
//    tf::Vector3 translation2 = object_tf.getOrigin() + m_relative_pose.getOrigin();
//    tf::Transform tf_ = tf::Transform(object_tf.getRotation()*m_relative_pose.getRotation(), translation2);

    m_publish_relative_pose = true;
    boost::thread thread(&Communication::publishRelativePoseTF,this,arm_pose_world_frame,arm_pose_world_frame,arm_pose_world_frame,arm_pose_world_frame,arm_pose_world_frame);
    //tf::StampedTransform goal_pose;
    //tf::TransformListener listener;
    //listener.waitForTransform("jaco_api_origin","jaco_tool_relative_pose",ros::Time(0),ros::Duration(3.0));
    //listener.lookupTransform("jaco_api_origin","jaco_tool_relative_pose",ros::Time(0),goal_pose);
    sleep(60);
    m_publish_relative_pose = false;
    thread.join();

    //m_jaco_ptr->moveToPoint(goal_pose);


}

void Communication::publishRelativePoseTF(tf::Transform arm,tf::Transform scene,tf::Transform model, tf::Transform final, tf::Transform object_centroid){
    static tf::TransformBroadcaster br;
    ros::Rate r(10);
    while(m_publish_relative_pose){

//        tf::StampedTransform object_model = tf::StampedTransform(model,ros::Time::now(),"camera_rgb_frame","tf_Object_model");
//        tf::StampedTransform arm_rel_pose = tf::StampedTransform(arm,ros::Time::now(),"tf_Object_model","tf_arm_position");
//        tf::StampedTransform final_arm_pose = tf::StampedTransform(final,ros::Time::now(),"camera_rgb_frame","tf_grasp_position");
//        tf::StampedTransform object_centroid_scene = tf::StampedTransform(object_centroid,ros::Time::now(),"camera_rgb_frame","tf_object_centroid");
//        tf::StampedTransform diff = tf::StampedTransform(scene,ros::Time::now(),"tf_object_centroid","tf_Object_model_test_align");
        //tf::StampedTransform arm_relative = tf::StampedTransform(relative_pose,ros::Time::now(),"camera_rgb_frame","jaco_tool_relative_pose");
//        br.sendTransform(arm_relative);

        tf::StampedTransform arm_pose_world = tf::StampedTransform(arm,ros::Time::now(),"camera_rgb_frame","tf_grasp_position");
        br.sendTransform(arm_pose_world);


//        br.sendTransform(arm_rel_pose);
//        br.sendTransform(final_arm_pose);
//        br.sendTransform(object_centroid_scene);
//        br.sendTransform(diff);
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
      To get the transform to work, we need to get the centroid directly from the ourcvfh algo.
      I made a new function that takes the centroid as a parameter.
      So, this way, we can set the origin of the tf
      */
    m_object_ex_ptr->m_transform_pc->clear();
    for(int i=0; i < tf_vector.size(); i++){
        Eigen::Matrix4f matrix = tf_vector.at(i);
        tf::Transform tf_ = tfFromEigen(matrix);
        tf::Vector3 vec2(centroidVec[i](2,0), -(centroidVec[i](0,0)), -(centroidVec[i](1,0)));
        tf_.setOrigin(vec2);

        // Send transform
        std::stringstream ss;
        ss << i;
        std::string str = "surfaceTransform_" + ss.str();
        br.sendTransform(tf::StampedTransform(tf_,ros::Time::now(),"camera_rgb_frame",str));

        PointT pt;
        pt.x = vec2.getX(); pt.y = vec2.getY(); pt.z = vec2.getZ();
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


void Communication::fillUserInterfaceWithObjectInfo(){

    int numberOfGrasps = 0;
    if(selected_object_index >= 0){
        // Object is recognized
        ObjectBd selected_object = m_api_ptr->retrieveObjectFromHistogram(selected_object_index);
        numberOfGrasps = selected_object.getArmPose().size();

        string sendString = "p_";
        string grasp_text = "";
        for(int i=0; i < numberOfGrasps; i++){
            stringstream ss;
            ss << (i+1);
            grasp_text += "Grasp_" + ss.str();
            if((i+1) < numberOfGrasps){
                grasp_text += ";" ;
            }
        }

        sendString += grasp_text;
        m_object_ex_ptr->publishToAndroidDevice(sendString);
        m_object_ex_ptr->publishToAndroidDevice("object_recon");
    }

    else{
        // Object is unrecognized
        m_object_ex_ptr->publishToAndroidDevice("object_not_recon");
    }

}


void Communication::saveToDBWithoutArmPose(){
    ros::Rate r(3);
    ros::NodeHandle node;
    while(node.ok()){
        if(m_object_ex_ptr->save_object_to_DB_FLAG){
            train(false,false);
            m_object_ex_ptr->save_object_to_DB_FLAG = false;
        }
        r.sleep();
    }
}


void Communication::testRecognition(){

    pcl::PointCloud<PointT>::Ptr input_pointcloud = m_object_ex_ptr->getObjectToGrasp();
    Eigen::Matrix4f input_to_model_transform;
    int object_index = m_object_ex_ptr->m_object_recognition.OURCVFHRecognition(input_pointcloud, m_api_ptr, input_to_model_transform);

    bool known_object = true;
    if(object_index < 0){
        known_object = false;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    ObjectBd obj;

    if(known_object){
        //Load object from Histogram position
        obj = m_api_ptr->retrieveObjectFromHistogram(object_index);
        object_pointcloud = obj.getPointCloud();

//        tf::Transform scene_to_model =  m_object_ex_ptr->m_object_recognition.tfFromEigen(input_to_model_transform);
//        tf::Vector3 translation = scene_to_model.getOrigin();
//        tf::Quaternion quat = scene_to_model.getRotation();

//        const Eigen::Vector3f offset(translation.getX(),translation.getY(),translation.getZ());
//        const Eigen::Quaternionf rotation(quat.getW(),quat.getX(),quat.getY(),quat.getZ());

        pcl::transformPointCloud(*input_pointcloud, *transformed_pointcloud, input_to_model_transform);
        *merged_pointcloud  = *transformed_pointcloud + *object_pointcloud;
    }

//    viewer_mutex.lock();
//    recognitionViewer->removeAllPointClouds();

//    // Input Pointcloud
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> yellow_color(input_pointcloud, 255, 255, 102);
//    recognitionViewer->addPointCloud<pcl::PointXYZRGB>(input_pointcloud, yellow_color, "input");
//    recognitionViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input");

//    // Model Pointcloud
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(object_pointcloud, 255, 0, 0);
//    recognitionViewer->addPointCloud<pcl::PointXYZRGB>(object_pointcloud, red_color, "model");
//    recognitionViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model");

//     //Transformed Pointcloud (input -> model)
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color(transformed_pointcloud, 0, 0, 255);
//    recognitionViewer->addPointCloud<pcl::PointXYZRGB>(transformed_pointcloud, blue_color, "transformed");
//    recognitionViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed");

////     Merged Pointcloud (new model)
////     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue(transformed_pointcloud, 20, 213, 130);
////     recognitionViewer->addPointCloud<pcl::PointXYZRGB>(transformed_pointcloud, blue, "transformed");
////     recognitionViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed");


//   // recognitionViewer->spinOnce(100);
//    viewer_mutex.unlock();


}

