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
    initialize_object_to_grasp = true;

    m_point_cloud_corner_ptr.reset(new pcl::PointCloud<PointT>);
    m_memory_point_cloud_corner_ptr.reset(new pcl::PointCloud<PointT>);

    m_pub_image = p_nh.advertise<sensor_msgs::Image>("/square_image",1);

    m_pub_android = p_nh.advertise<std_msgs::String>("/android_listener",1);


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
        object_to_grasp = object_vector[index_to_grasp];
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
            object_to_grasp = object_vector[index_to_grasp];
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
            change_pixel_color(p_array, i, p_bottom_left.y,0,255,0);
        }
        //draw the left line
        for(int i = p_top_left.y; i <= p_bottom_left.y; i++)
        {
            change_pixel_color(p_array, p_top_left.x, i,0,0,255);
        }
        //draw the right line
        for(int i = p_top_right.y; i <=p_bottom_right.y; i++)
        {
            change_pixel_color(p_array, p_top_right.x, i,255,255,0);
        }
    }
}

//-----------------------------------------------------------------------------------------------------//
void ObjectExtractor::image_processing(pcl::PointCloud<PointT>::Ptr p_point_cloud_corner, sensor_msgs::Image p_image_input)
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
    for(int  i =0; i < p_image_input.data.size(); i ++)
    {
        m_image_memory.data.push_back(p_image_input.data.at(i));
    }
}

//---------------------------------------------------------------------------------------------------------------//
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
void ObjectExtractor::coordinate_processing(const float p_coordinate[])
{
    //a  concerver dans un attribut ou autre element pour pouvoir l'utiliser apres
    int position_in_vector = position_finder_vector(p_coordinate, *m_memory_point_cloud_corner_ptr, m_memory_distance_vector);
    //debug response to android
    std_msgs::String send_string;
    send_string.data = "p_un;deux";
    m_pub_android.publish(send_string);
    send_string.data = "object_recon";
    m_pub_android.publish(send_string);
}

//----------------------------------------------------------------------------------------------------------------------------//
void ObjectExtractor::point_cloud_processing()
{
    image_processing(m_point_cloud_corner_ptr, m_image_received_input);
    *m_memory_point_cloud_corner_ptr = *m_point_cloud_corner_ptr;
    m_memory_distance_vector = m_distance_vector;
    m_memory_point_cloud = object_vector;
}


//---------------------------------------------------------------------------------------------------------------------------------------//
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

