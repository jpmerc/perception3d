#include <objectExtractor.h>
typedef pcl::PointXYZRGB PointT;


// -------------------------------------------------------------------------------------------------------- //
ObjectExtractor::ObjectExtractor(bool showViewer, ros::NodeHandle p_nh){
    cloud.reset(new pcl::PointCloud<PointT>);
    pclViewer.reset(new pcl::visualization::PCLVisualizer ("3DViewer"));
    pclViewer->registerKeyboardCallback(&ObjectExtractor::keyboard_callback,*this, (void*)&pclViewer);
    set_showUI(showViewer);
    l_count = 0;
    index_to_grasp = 0;
    object_to_grasp.reset(new pcl::PointCloud<PointT>);
    tracked_object_centroid.reset(new pcl::PointCloud<PointT>);
    initialize_object_to_grasp = true;

    m_point_cloud_corner_ptr.reset(new pcl::PointCloud<PointT>);
    m_memory_point_cloud_corner_ptr.reset(new pcl::PointCloud<PointT>);

    m_pub_image = p_nh.advertise<sensor_msgs::Image>("/square_image",1);

    m_pub_android = p_nh.advertise<std_msgs::String>("/android_listener",1);

    NumberOfSnapshots = 0;
    directory = "/home/robot/rosWorkspace/src/perception3d/";

    m_transform_pc.reset(new pcl::PointCloud<PointT>);

    setPCLViewer();
}

// -------------------------------------------------------------------------------------------------------- //
void ObjectExtractor::extraction_callback(const pcl::PCLPointCloud2ConstPtr& input){

    m_point_cloud_received = true;

    m_corner_cloud.clear();
    m_object_vector_2d.clear();


    pcl::PointCloud<PointT>::Ptr objects(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*input,*objects);
    cloud = objects;

    //Extract objects
    if(!object_vector.empty()){object_vector.clear();}
    object_vector = segment_objects(objects,0.02,200,15000);


    // Initialize object to grasp
    if((!object_vector.empty()) && initialize_object_to_grasp){
        refreshObjectCentroid();
        initialize_object_to_grasp = false;
    }

    if(showUI){
        printToPCLViewer();
    }

    for(int i = 0; i < object_vector.size(); i++)
    {
        projection2d_pointCloud(*(object_vector.at(i)), m_object_vector_2d);
    }

    m_point_cloud_corner_ptr->clear();
    for(int i = 0; i < object_vector.size(); i++)
    {
        Eigen::Matrix<float,4,1> matrix = compute_centroid_point(*(object_vector.at(i)));
        m_distance_vector.push_back(compute_distance_from_kinect(matrix));
        Eigen::Matrix<float,4,1> matrix_2d = projection2d_matrix(matrix);
        point_cloud_limit_finder(matrix_2d, m_object_vector_2d.at(i));
    }

}

// -------------------------------------------------------------------------------------------------------- //
void ObjectExtractor::printToPCLViewer(){
    pclViewer->removeAllPointClouds();

    for(int i=0; i < object_vector.size(); i++){
        pcl::PointCloud<PointT>::Ptr pc = object_vector[i];
        pcl::visualization::PointCloudColorHandlerRandom<PointT> randColor(pc);
        std::stringstream ss;
        ss << i;
        std::string ind = ss.str();
        std::string pc_name = "object_" + ind;
        pclViewer->addPointCloud<PointT>(pc,randColor,pc_name);
        pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pc_name);
    }

    //    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(object_to_grasp);
    //    pclViewer->addPointCloud<PointT>(object_to_grasp,rgb,"object_to_grasp");
    //    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object_to_grasp");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow_color(object_to_grasp, 255, 255, 102);
    pclViewer->addPointCloud<PointT>(object_to_grasp,yellow_color,"object_to_grasp");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object_to_grasp");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> centroid_color (tracked_object_centroid, 255, 0, 255);
    pclViewer->addPointCloud<PointT> (tracked_object_centroid, centroid_color, "centroid");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 12, "centroid");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> tf_color (m_transform_pc, 255, 0, 255);
    pclViewer->addPointCloud<PointT> (m_transform_pc, tf_color, "tf");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 12, "tf");

}

// -------------------------------------------------------------------------------------------------------- //
void ObjectExtractor::keyboard_callback(const pcl::visualization::KeyboardEvent &event, void* viewer_void){
    l_count = l_count + 1;
    if(l_count < 2){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym () == "p"){
            if(showUI){
                showUI=false;
            }
            else{
                showUI=true;
            }
        }
        else if(event.getKeySym () == "s"){
            if(index_to_grasp+1 < object_vector.size()){
                index_to_grasp++;
            }
            else{
                index_to_grasp = 0;
            }
            // object_to_track = addNormalsToPointCloud(object_vector[index_to_track]);
            refreshObjectCentroid();

        }

        else if(event.getKeySym () == "m"){

            std::string base_filename = "snapshot";
            char new_filename[250];
            if(NumberOfSnapshots > 0){
                sprintf(new_filename,"%s%d.pcd",base_filename.c_str(),NumberOfSnapshots+1);
                NumberOfSnapshots++;
            }
            else{
                sprintf(new_filename,"%s.pcd",base_filename.c_str());
                NumberOfSnapshots++;
            }
            std::string filename = std::string(new_filename);
            std::string path = directory + filename;

            std::cout << filename << std::endl;

            pcl::io::savePCDFileASCII(path,*object_to_grasp);
            std::stringstream ss;
            ss << directory << base_filename << NumberOfSnapshots << ".png";
            std::string path2 = ss.str();
            viewer->saveScreenshot("/home/robot/snapshot_jean.png");

        }
    }
    else{
        l_count = 0;
    }

}

// -------------------------------------------------------------------------------------------------------- //
std::vector<pcl::PointCloud<PointT>::Ptr> ObjectExtractor::segment_objects(pcl::PointCloud<PointT>::Ptr cloud_input, double tolerance, int minClusterSize, int maxClusterSize){
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (tolerance); // 2cm
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_input);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_input);
    ec.extract (cluster_indices);

    // returns a vector of all the objects
    std::vector<pcl::PointCloud<PointT>::Ptr> object_vector_temp;
    for (int i=0; i<cluster_indices.size(); i++){
        pcl::PointIndices cloud_indices = cluster_indices.at(i);
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        cloud_cluster = extract_object_from_indices(cloud_input,cloud_indices);
        cloud_cluster->height = 1;
        cloud_cluster->width = cloud_cluster->size();
        object_vector_temp.push_back(cloud_cluster);
    }

    return object_vector_temp;
}

// -------------------------------------------------------------------------------------------------------- //
pcl::PointCloud<PointT>::Ptr ObjectExtractor::extract_object_from_indices(pcl::PointCloud<PointT>::Ptr cloud_input,pcl::PointIndices object_indices){
    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (int j=0; j<object_indices.indices.size(); j++){
        cloud_cluster->points.push_back (cloud_input->points[object_indices.indices[j]]);
    }
    return cloud_cluster;
}

// -------------------------------------------------------------------------------------------------------- //
// Get showUI
bool ObjectExtractor::get_showUI(){
    return showUI;
}

// -------------------------------------------------------------------------------------------------------- //
// Set the value of showUI
void ObjectExtractor::set_showUI(bool show){
    showUI = show;
}

// -------------------------------------------------------------------------------------------------------- //
void ObjectExtractor::toggle_showUI(){
    set_showUI(!get_showUI());
    setPCLViewer();
}

// -------------------------------------------------------------------------------------------------------- //
void ObjectExtractor::setPCLViewer(){
    if(showUI){
        pclViewer->setBackgroundColor (0, 0, 0);
        pclViewer->initCameraParameters ();
        pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(800,450);
        renderWindow->Render();

    }
    else{ // Not very clean, but only solution I found to hide pclViewer
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(1,1);
        renderWindow->Render();
    }
}

// -------------------------------------------------------------------------------------------------------- //
Eigen::Vector4f ObjectExtractor::getGraspCentroid(){
    Eigen::Vector4f c;
    pcl::compute3DCentroid<PointT>(*object_to_grasp,c);
    return c;
}

// -------------------------------------------------------------------------------------------------------- //
pcl::PointCloud<PointT>::Ptr ObjectExtractor::getObjectToGrasp(){
    return object_to_grasp;
}

// -------------------------------------------------------------------------------------------------------- //
tf::StampedTransform ObjectExtractor::getCentroidPositionRGBFrame(){
    Eigen::Vector4f object_pose = getGraspCentroid();
    //cout << "object pose : [" <<  object_pose[2] << ", " << -object_pose[0] << ", " << -object_pose[1] << "]" << endl;
    tf::Pose tf_pose;
    tf_pose.setIdentity();
    tf_pose.setOrigin(tf::Vector3(object_pose[2],-object_pose[0],-object_pose[1]));
/* Comment to debug the corner padding
    cout << "---Detected Object Centroid---" << endl;
    cout << "x: " <<  object_pose[2] << endl;
    cout << "y: " << -object_pose[0] << endl;
    cout << "z: " << -object_pose[1] << endl;
    cout << endl;
*/
    return tf::StampedTransform(tf_pose, ros::Time::now(), "camera_rgb_frame", "detected_object_centroids");
}

//---------------------------------------------------------------------------------------------------------//
void ObjectExtractor::callback_rgb_camera(const sensor_msgs::Image& p_input)
{
    m_image_received_input = p_input;
    m_image_received_input.header = p_input.header;
}

//-------------------------------------------------------------------------------------------------------//
float ObjectExtractor::compute_distance_from_kinect(Eigen::Matrix<float, 4, 1> p_matrix)
{
    pcl::PointXYZ camera_origin(0,0,0);
    pcl::PointXYZ object_position(p_matrix(0,0), p_matrix(1,0), p_matrix(3,0));
    float distance = pcl::euclideanDistance(camera_origin, object_position);

    return distance;
}

Eigen::Matrix<float,4,1> ObjectExtractor::compute_centroid_point(const pcl::PointCloud<PointT>& p_point_cloud)
{
    pcl::ConstCloudIterator<PointT> it(p_point_cloud);
    Eigen::Matrix< float, 4, 1 > matrix;
    pcl::compute3DCentroid(it, matrix);
    return matrix;
}

//--------------------------------------------------------------------------------------------------------//
/*
  Find the limit of the point cloud.  Will find the corner of the point cloud after.
  param[in] p_matrix the centroid matrix of the point cloud.
  param[in] p_ptr the point cloud to search in
  */
void ObjectExtractor::point_cloud_limit_finder (Eigen::Matrix<float, 4, 1> p_matrix, pcl::PointCloud<PointT>::Ptr p_ptr)
{
    float x = p_matrix(0,0);
    float y = p_matrix(1,0);
    float z = p_matrix(2,0);
    pcl::PointXYZRGB left(255, 0, 0);
    left.x = x;
    left.y = y;
    left.z = z;
    pcl::PointXYZRGB right(0,255,0);
    right.x = x;
    right.y = y;
    right.z = z;
    pcl::PointXYZRGB top(0,0,255);
    top.x = x;
    top.y = y;
    top.z = z;
    pcl::PointXYZRGB bottom(0,0,0);
    bottom.x = x;
    bottom.y = y;
    bottom.z = z;

    for(int i = 0; i < p_ptr->size(); i++)
    {
        pcl::PointXYZRGB point(255,0,0);
        point.x = p_ptr->at(i).x;
        point.y = p_ptr->at(i).y;
        point.z = p_ptr->at(i).z;
        if(point.x > left.x)
            left = point;
        else if (point.x < right.x)
            right = point;
        if(point.y < top.y)
            top = point;
        else if(point.y > bottom.y)
            bottom = point;
    }

    find_corner(left,right,top,bottom,m_point_cloud_corner_ptr);
}

//------------------------------------------------------------------------------------------------------------------//
/*
  Funtion to find the corner from 4 point.
  param[in] p_left the more left point.
  param[in] p_right the more right point
  param[in] p_top  more top point
  param[in] p_bottom the more bottom point
  param[out] p_point_cloud_output the point cloud to keep the point.  The order is top left, top right,
  bottom left, bottom right.
  */
void ObjectExtractor::find_corner(const PointT& p_left, const PointT& p_right, const PointT& p_top, const PointT& p_bottom, pcl::PointCloud<PointT>::Ptr& p_point_cloud_output)
{
    PointT top_right(255,0,0);
    top_right.x = p_left.x;
    top_right.y = p_top.y;
    top_right.z = 1;

    PointT top_left(0,255,0);
    top_left.x = p_right.x;
    top_left.y = p_top.y;
    top_left.z = 1;

    PointT bottom_right(0,0,255);
    bottom_right.x = p_left.x;
    bottom_right.y = p_bottom.y;
    bottom_right.z = 1;

    PointT bottom_left(255,255,0);
    bottom_left.x = p_right.x;
    bottom_left.y = p_bottom.y;
    bottom_left.z = 1;

    p_point_cloud_output->points.push_back(top_left);
    p_point_cloud_output->points.push_back(top_right);
    p_point_cloud_output->points.push_back(bottom_left);
    p_point_cloud_output->points.push_back(bottom_right);
}
//------------------------------------------------------------------------------------------------//
/*
  Funtion to see if the corner is in padding zone.
  param[in] p_memoryCloud_ptr the old point cloud to search in.
  param[out] p_cloud_ptr the point cloud that containt the new point cloud.
  pram[in] p_distance the thresold distance to use.
  */
void ObjectExtractor::paddingCorner(pcl::PointCloud<PointT>::Ptr p_memoryCloud_ptr,
                                    pcl::PointCloud<PointT>::Ptr p_cloud_ptr,
                                    int p_distance)
{
    for(int i = 0; i < p_cloud_ptr->size(); i += 4)
    {
        PointT topLeft = p_cloud_ptr->at(i);
        PointT topRight = p_cloud_ptr->at(i+1);
        PointT bottomLeft = p_cloud_ptr->at(i+2);
        PointT bottomRight = p_cloud_ptr->at(i+3);
        for(int j = 0; j < p_memoryCloud_ptr->size(); j+=4)
        {
            PointT topLeftM = p_memoryCloud_ptr->at(j);
            PointT topRightM = p_memoryCloud_ptr->at(j+1);
            PointT bottomLeftM = p_memoryCloud_ptr->at(j+2);
            PointT bottomRightM = p_memoryCloud_ptr->at(j+3);

            if(distancePadding(topLeft, topLeftM, p_distance) and
                    distancePadding(topRight, topRightM, p_distance) and
                    distancePadding(bottomLeft, bottomLeftM, p_distance) and
                    distancePadding(bottomRight, bottomRightM, p_distance))
            {
                p_cloud_ptr->at(i) = topLeftM;
                p_cloud_ptr->at(i+1) = topRightM;
                p_cloud_ptr->at(i+2) = bottomLeftM;
                p_cloud_ptr->at(i+3) = bottomRightM;
                break;
            }
            else
                break;
        }
    }
}

//------------------------------------------------------------------------------------------------//
/*
  Function that look if the distance from 2 point are lower than de p_distance.  Search for a radius.
  param[in] p_point the point the we use.
  param[in] p_mPoint the point we have in memory
  param[in] p_distance the maximum distance.
  */
bool ObjectExtractor::distancePadding(PointT p_point, PointT p_mPoint, int p_distance)
{
    float deltaX = p_point.x - p_mPoint.x;
    float deltaY = p_point.y - p_mPoint.y;

    float c = sqrt(pow(deltaX, 2) + pow(deltaY,2));
    bool validation = false;

    if(c <= p_distance)
    {
        validation = true;
    }

    return validation;
}


//------------------------------------------------------------------------------------------------------------//
Eigen::Matrix<float,4,1> ObjectExtractor::projection2d_matrix(const Eigen::Matrix<float,4,1>& p_matrix)
{
    Eigen::Matrix<float,4,1> matrix_2d;
    matrix_2d(0,0) = (((p_matrix(0,0) * 525)/p_matrix(2,0))+320);
    matrix_2d(1,0) = (((p_matrix(1,0) * 525)/p_matrix(2,0))+240);
    matrix_2d(2,0) = p_matrix(2,0)/p_matrix(2,0);
    matrix_2d(3,0) = p_matrix(3,0);
    return matrix_2d;
}

//------------------------------------------------------------------------------------------------------------------//
/*
  Function to project the a point cloud in the 2d.  Its now set with the kinect focal.
  param[in] p_point_cloud the point cloud input.
  param[in] p_vector_output a vector that containt the point cloud in 2d.  Used this way to project
  the object in 2d.
  */
void ObjectExtractor::projection2d_pointCloud(const pcl::PointCloud<PointT>& p_point_cloud, std::vector<pcl::PointCloud<PointT>::Ptr>& p_vector_output)
{
    pcl::PointCloud<PointT>::Ptr point_cloud_2d(new pcl::PointCloud<PointT>);
    for(int i = 0; i < p_point_cloud.size(); i++)
    {
        PointT point_3d = p_point_cloud.at(i);
        PointT point(1,1,1);
        point.x = (((point_3d.x * 525) / point_3d.z)+320);
        point.y = (((point_3d.y * 525) / point_3d.z)+240);
        point.z = point_3d.z / point_3d.z;
        point_cloud_2d->push_back(point);
    }
    p_vector_output.push_back(point_cloud_2d);
}

//-------------------------------------------------------------------------------------------------//
/*
  Change the pixel color of a RGB image
  param[in] p_array the image array
  param[in] p_x the x pixel coordinate
  param[in] p_y the y pixel coordinate
  param[in] p_b the blue color [0-255]
  param[in] p_g the green color [0-255]
  param[in] p_r the red color [0-255]
  */
void ObjectExtractor::change_pixel_color(std::vector<unsigned char>& p_array, int p_x, int p_y, int p_b, int p_g, int p_r)
{
    if(p_x < 0)
        p_x = 0;
    else if(p_x > 640)
        p_x = 640;
    if(p_y < 0)
        p_y = 0;
    if(p_y > 480)
        p_y = 480;


    int row = 3*p_x + 1920*p_y;
    p_array[row] = p_b;
    p_array[row + 1] = p_g;
    p_array[row + 2] = p_r;
}

//------------------------------------------------------------------------------------------------------------------------//
/*
  Function that draw a square in a rgb image.
  param[in] p_array a vector that containt the rgb image.
  param[in] p_top_left the top left point
  param[in] p_top_right the top right point
  param[in] p_bottom_left the bottom left point
  param[in] p_bottom_right the bottom right point
  */
void ObjectExtractor::draw_square(std::vector<unsigned char>& p_array, PointT p_top_left, PointT p_top_right, PointT p_bottom_left, PointT p_bottom_right)
{
    if(p_array.size() == 921600)
    {
        //draw the upper line
        for(int i = p_top_left.x; i <= p_top_right.x; i++)
        {
            change_pixel_color(p_array, i, p_top_left.y);
        }
        //draw the down line
        for(int i = p_bottom_left.x; i <= p_bottom_right.x; i++)
        {
            change_pixel_color(p_array, i, p_bottom_left.y);
        }
        //draw the left line
        for(int i = p_top_left.y; i <= p_bottom_left.y; i++)
        {
            change_pixel_color(p_array, p_top_left.x, i);
        }
        //draw the right line
        for(int i = p_top_right.y; i <=p_bottom_right.y; i++)
        {
            change_pixel_color(p_array, p_top_right.x, i);
        }
    }
}

//-----------------------------------------------------------------------------------------------------//
/*
  Fonction used to change the rgb image received by the camera when a point cloud is received.
  Will draw the square arount the object.
  param[in] p_point_cloud_corner the corner point cloud.
  param[out] p_image_input the image received that match the point cloud.
  */
void ObjectExtractor::image_processing(pcl::PointCloud<PointT>::Ptr p_point_cloud_corner, sensor_msgs::Image& p_image_input)
{
    int compteur = 0;
    PointT top_left(255,0,0);
    PointT top_right(255,0,0);
    PointT bottom_left(255,0,0);
    PointT bottom_right(255,0,0);
    for(int i = 0; i < p_point_cloud_corner->size(); i++)
    {
        switch(compteur)
        {
        case(0):top_left = p_point_cloud_corner->at(i); compteur++; break;
        case(1):top_right = p_point_cloud_corner->at(i);compteur++;break;
        case(2):bottom_left = p_point_cloud_corner->at(i);compteur++;break;
        case(3):bottom_right = p_point_cloud_corner->at(i);
            draw_square(p_image_input.data, top_left, top_right, bottom_left, bottom_right);
            compteur = 0;
            break;
        }
    }
    m_image_memory.data.clear();
    m_image_memory.header = p_image_input.header;
    m_image_memory.encoding = p_image_input.encoding;
    m_image_memory.width = p_image_input.width;
    m_image_memory.height = p_image_input.height;
    m_image_memory.step = p_image_input.step;
    m_image_memory.data = m_image_received_input.data;
}

//---------------------------------------------------------------------------------------------------------------//
/*
  Find if the coordinate correspond to a object or not.
  param[in] p_coordinate the coordinate received
  param[in] p_point_cloud_corner the memory point cloud corner.
  param[in] p_distance_vector the memory distance of the object.
  return the position of the object or -1 if the coordinate dont belong to an object.
  */
int ObjectExtractor::position_finder_vector(const float p_coordinate[], const pcl::PointCloud<PointT>& p_point_cloud_corner, const std::vector<float> p_distance_vector)
{
    std::vector<int> position_vector;
    for(int i = 0; i < p_point_cloud_corner.size(); i += 4)
    {
        PointT point_temps_top_left = p_point_cloud_corner.at(i);
        PointT point_temps_bottom_right = p_point_cloud_corner.at(i+3);
        if(p_coordinate[0] >= point_temps_top_left.x and p_coordinate[0] <= point_temps_bottom_right.x)
        {
            if(p_coordinate[1] >= point_temps_top_left.y and p_coordinate[1] <= point_temps_bottom_right.y)
            {
                position_vector.push_back(i/4);
            }
        }
    }
    if(position_vector.size() > 0)
    {
        float distance = position_vector.at(0);
        int indice = position_vector.at(0);
        for(int i = 0; i < position_vector.size(); i++)
        {
            if(p_distance_vector.at(position_vector.at(i)) < distance)
            {
                distance = position_vector.at(i);
                indice = position_vector.at(i);
            }
        }
        return indice;
    }
    return -1;
}


//----------------------------------------------------------------------------------------------------------------------//
/*
  The fonction that process the coordinate send by the tablet.
  param[in] p_coordinate the coordinate received
  param[in] p_bd the signature db
  */
int ObjectExtractor::coordinate_processing(const float p_coordinate[],
                                           pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd)
{
    int position_in_vector = position_finder_vector(p_coordinate,*m_memory_point_cloud_corner_ptr,m_memory_distance_vector);
    if(position_in_vector != -1)
    {/*
        //fait la reconnaisance d'object avec le point cloud qui se trouve a la position
        int positionVectorObject = m_object_recognition.object_recon(object_vector.at(position_in_vector)
                                                                     , p_bd);*/
        //debug response to android
        std_msgs::String send_string;
        send_string.data = "p_un;deux";
        m_pub_android.publish(send_string);
        send_string.data = "object_recon";
        m_pub_android.publish(send_string);
        //return positionVectorObject;
    }
    if(position_in_vector == -1)
    {
        std::cout << "no object clicked" << std::endl;
        std_msgs::String sendString;
        sendString.data = "no_object";
        m_pub_android.publish(sendString);
    }
}

//----------------------------------------------------------------------------------------------------------------------------//
void ObjectExtractor::point_cloud_processing()
{
    //function pour faire le padding sur les corners avant de les dessiner sur l'image
    paddingCorner(m_memory_point_cloud_corner_ptr, m_point_cloud_corner_ptr, 10);
    image_processing(m_point_cloud_corner_ptr, m_image_received_input);
    *m_memory_point_cloud_corner_ptr = *m_point_cloud_corner_ptr;
    m_memory_distance_vector = m_distance_vector;
    m_memory_point_cloud = object_vector;
}


//---------------------------------------------------------------------------------------------------------------------------------------//
/*
  Call all the calback depend of what the kinect have send.  Image or point cloud.
  */
void ObjectExtractor::spin_once()
{
    if(m_point_cloud_received)
    {
        point_cloud_processing();
        m_pub_image.publish(m_image_received_input);
    }
    else
    {
        m_pub_image.publish(m_image_memory);
    }
    m_point_cloud_received = false;
}

void ObjectExtractor::refreshObjectCentroid(){
    // Change object to grasp
    object_to_grasp = object_vector[index_to_grasp];

    // Refresh the position of the object centroid
    Eigen::Vector4f c = getGraspCentroid();
    PointT pt;
    pt.x = c[0]; pt.y = c[1]; pt.z = c[2];
    tracked_object_centroid->clear();
    tracked_object_centroid->push_back(pt);
}


