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

    ros::NodeHandle node;
    ObjectToGrasp_publisher_ = node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/grasp_object",1);
    publish_objectToGrasp_thread_ = boost::thread(&Communication::publish_objectToGrasp,this);

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


    // Find TF in jaco coordinate system and send command to MoveIt
    m_publish_relative_pose = true;

    tf::TransformListener listener;

    //---- Grasp Position
    boost::thread thread_grasp(&Communication::publishGraspTF,this,arm_pose_world_frame);
    ros::Rate freq(5);
    bool tf_ready = false;
    while(!tf_ready){
        freq.sleep();
        tf_ready = listener.waitForTransform("root","tf_grasp_position",ros::Time(0),ros::Duration(5.0));
    }
    tf::StampedTransform grasp_pose;
    listener.lookupTransform("root","tf_grasp_position",ros::Time(0),grasp_pose);


    //---- Pre-grasp position #1
    boost::thread thread_pre_grasp1(&Communication::publishPreGraspTF, this, 0.10, "tf_pre_grasp_position1");
    tf_ready = false;
    while(!tf_ready){
        freq.sleep();
        tf_ready = listener.waitForTransform("root","tf_pre_grasp_position1",ros::Time(0),ros::Duration(5.0));
    }
    tf::StampedTransform pre_grasp_pose1;
    listener.lookupTransform("root","tf_pre_grasp_position1",ros::Time(0),pre_grasp_pose1);


    //---- Pre-grasp position #2
    boost::thread thread_pre_grasp2(&Communication::publishPreGraspTF, this, 0.05, "tf_pre_grasp_position2");
    tf_ready = false;
    while(!tf_ready){
        freq.sleep();
        tf_ready = listener.waitForTransform("root","tf_pre_grasp_position2",ros::Time(0),ros::Duration(5.0));
    }
    tf::StampedTransform pre_grasp_pose2;
    listener.lookupTransform("root","tf_pre_grasp_position2",ros::Time(0),pre_grasp_pose2);


    // Add listeners for pose in jaco api referential
    tf::StampedTransform api_referential_tf;
    listener.lookupTransform("jaco_link_hand","jaco_tool_position",ros::Time(0),api_referential_tf);
    tf::Transform pre_grasp_api1 = pre_grasp_pose1 * api_referential_tf;
    tf::Transform pre_grasp_api2 = pre_grasp_pose2 * api_referential_tf;
    tf::Transform grasp_api = grasp_pose * api_referential_tf;

    // Add a thread and publish the tf to help debugging
    boost::thread thread1(&Communication::publishTF, this, pre_grasp_api1, "root", "tf_pre_grasp_api_1");
    boost::thread thread2(&Communication::publishTF, this, pre_grasp_api2, "root", "tf_pre_grasp_api_2");
    boost::thread thread3(&Communication::publishTF, this, grasp_api, "root", "tf_grasp_api");

    std::cout << "The arm will start moving in 5 seconds..." << std::endl;
    sleep(5);
    m_publish_relative_pose = false;
    thread_grasp.join();
    thread_pre_grasp1.join();
    thread_pre_grasp2.join();
    thread1.join();
    thread2.join();
    thread3.join();

    std::cout << "now!" << std::endl;

    bool succeeded = m_jaco_ptr->moveitPlugin(pre_grasp_pose1);

    // REMOVE ALL THE SLEEPS SOON!
    sleep(10);

    if(succeeded){
        std::cout << "Moving the arm to pre-grasp position!" << std::endl;
//        m_jaco_ptr->moveToPoint(pre_grasp_api1);
//        sleep(10);
//        m_jaco_ptr->moveToPoint(pre_grasp_api2);
//        sleep(10);
//        m_jaco_ptr->moveToPoint(grasp_api);
//        sleep(10);
//        m_jaco_ptr->close_fingers();
//        sleep(10);
//        m_jaco_ptr->move_up(0.1);
//        sleep(10);
    }
}

void Communication::publishGraspTF(tf::Transform arm){
    static tf::TransformBroadcaster br;
    ros::Rate r(10);
    while(m_publish_relative_pose){
        tf::StampedTransform arm_pose_world = tf::StampedTransform(arm,ros::Time::now(),"camera_rgb_frame","tf_grasp_position");
        br.sendTransform(arm_pose_world);
        r.sleep();
    }
}

void Communication::publishPreGraspTF(double distance, std::string target_frame){
    static tf::TransformBroadcaster br;
    ros::Rate r(10);

    tf::Transform pre_grasp_tf;
    pre_grasp_tf.setIdentity();
    pre_grasp_tf.setOrigin(tf::Vector3(0,0,distance));

    while(m_publish_relative_pose){
        tf::StampedTransform pre_grasp_pose_world = tf::StampedTransform(pre_grasp_tf, ros::Time::now(), "tf_grasp_position", target_frame);
        br.sendTransform(pre_grasp_pose_world);
        r.sleep();
    }
}


void Communication::publishTF(tf::Transform in_tf, std::string src, std::string target){
    static tf::TransformBroadcaster br;
    ros::Rate r(10);

    while(m_publish_relative_pose){
        tf::StampedTransform st_tf = tf::StampedTransform(in_tf, ros::Time::now(), src, target);
        br.sendTransform(st_tf);
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


void Communication::publish_objectToGrasp_moveitFormat(){
    ros::Rate r(1);
    ros::NodeHandle node;
    static tf::TransformBroadcaster br;
    Eigen::Matrix4f camToWorldMatrix; camToWorldMatrix.setZero();camToWorldMatrix(0,2) = 1; camToWorldMatrix(1,0) = -1; camToWorldMatrix(2,1) = -1; camToWorldMatrix(3,3) = 1;

    while(node.ok()){
        pcl::PointCloud<PointT>::Ptr object_pc = m_object_ex_ptr->getObjectToGrasp();
        pcl::PointCloud<PointT>::Ptr object_pc_transformed(new pcl::PointCloud<PointT>);
//        Eigen::Vector4f c = m_object_ex_ptr->getGraspCentroid(); c(3)=1;
//        tf::Transform simpleTF;
        tf::StampedTransform camToJacoTf;
        tf::TransformListener listener;
        bool tf_ready = listener.waitForTransform("camera_rgb_frame","root",ros::Time(0),ros::Duration(5.0));
        if(object_pc->size() > 0 && tf_ready){
            listener.lookupTransform("camera_rgb_frame","root",ros::Time(0),camToJacoTf);
            Eigen::Matrix4f camToJacoMatrix;
            pcl_ros::transformAsMatrix(camToJacoTf,camToJacoMatrix);

            Eigen::Matrix4f combinedMatrix = camToJacoMatrix.inverse() * camToWorldMatrix;
//            Eigen::Vector4f result = combinedMatrix * c;
//            Eigen::Matrix4f res;  res(0,3) = result(0); res(1,3) = result(1); res(2,3) = result(2); res(3,3) = 1;
//            simpleTF = tfFromEigen(res);
//            br.sendTransform(tf::StampedTransform(simpleTF, ros::Time::now(), "root", "object_centroid_test"));

            pcl::transformPointCloud(*object_pc, *object_pc_transformed, combinedMatrix);
//            std::cout << "old : " << object_pc->at(100) << std::endl;
//            std::cout << "new : " << object_pc_transformed->at(100) << std::endl;
            ObjectToGrasp_publisher_.publish(object_pc_transformed);
        }
        r.sleep();
    }
}

void Communication::publish_objectToGrasp(){
    ros::Rate r(10);
    ros::NodeHandle node;

    while(node.ok()){
        pcl::PointCloud<PointT>::Ptr object_pc = m_object_ex_ptr->getObjectToGrasp();
        ObjectToGrasp_publisher_.publish(object_pc);
        r.sleep();
    }
}


