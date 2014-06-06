#include <ros/ros.h>
// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>

#include <pcl/cloud_iterator.h>
#include<pcl/common/centroid.h>
#include<pcl/common/distances.h>
#include<pcl_ros/point_cloud.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/UInt32MultiArray.h>
#include<boost/thread/mutex.hpp>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterTracker<PointT, ParticleT> ParticleFilter;

ros::Publisher pub;
ros::Publisher PUB2;
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr object_to_track(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr tracked_object(new pcl::PointCloud<PointT>);
boost::shared_ptr<ParticleFilter> tracker_(new ParticleFilter);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_vector;
std::vector<pcl::PointCloud<PointT>::Ptr> OBJECT_VECTOR_2D;
pcl::PointCloud<PointT> CORNER_CLOUD;
pcl::PointCloud<PointT>::Ptr POINT_CLOUD_CORNER(new pcl::PointCloud<PointT>);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> MEMORY_POINT_CLOUD;
pcl::PointCloud<PointT>::Ptr MEMORY_POINT_CLOUD_CORNER_PTR(new pcl::PointCloud<PointT>);
bool POINT_CLOUD_RECEIVED = false;
sensor_msgs::Image IMAGE_RECEIVED_INPUT;
sensor_msgs::Image IMAGE_MEMORY;

boost::mutex MTX;
int index_to_track = 0;
bool COORDINATE_RECEIVED = false;
std_msgs::UInt32MultiArray COORDINATE_USER_SENDED;


// PCL Viewer
bool showUI = true;
int l_count = 0;
bool initialize_object_to_track = true;
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer 2"));

//Function declarations
pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_object_from_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,pcl::PointIndices object_indices);
void find_corner(const PointT& p_left, const PointT& p_right, const PointT& p_top, const PointT& p_bottom, pcl::PointCloud<PointT>::Ptr& p_point_cloud_output);

void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    //pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"source cloud");
    //pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    for(int i=0; i < object_vector.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = object_vector[i];
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> randColor(pc);
        std::stringstream ss;
        ss << i;
        std::string ind = ss.str();
        std::string pc_name = "object_" + ind;
        pclViewer->addPointCloud<pcl::PointXYZRGB>(pc,randColor,pc_name);
        pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pc_name);
    }

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> yellow_color(tracked_object, 255, 255, 102);
//    pclViewer->addPointCloud<pcl::PointXYZRGB>(tracked_object,yellow_color,"tracked cloud");
//    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tracked cloud");
}

void initTracking(){
    // based on PCL apps : https://github.com/PointCloudLibrary/pcl/blob/master/apps/src/openni_tracking.cpp

    //Particle filter with 8 threads
    boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT> > tracker(new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT> (8));

    //TODO check the meaning of parameters
    tracker->setMaximumParticleNum (500);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;
    tracker->setBinSize (bin_size);
    tracker_ = tracker;

    // Setup covariances and means to initialize particle filter
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (400);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);
    tracker_->setMinIndices (100);
    //tracker->setUseChangeDetector(false);


    //Setup Coherence
    pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT>::Ptr(new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT> ());
    boost::shared_ptr<pcl::tracking::DistanceCoherence<PointT> > distance_coherence = boost::shared_ptr<pcl::tracking::DistanceCoherence<PointT> > (new pcl::tracking::DistanceCoherence<PointT> ());
    boost::shared_ptr<pcl::tracking::HSVColorCoherence<PointT> > color_coherence = boost::shared_ptr<pcl::tracking::HSVColorCoherence<PointT> > (new pcl::tracking::HSVColorCoherence<PointT> ());
    color_coherence->setWeight (0.1);
    coherence->addPointCoherence (distance_coherence);
    coherence->addPointCoherence (color_coherence);
    boost::shared_ptr<pcl::search::Octree<PointT> > search (new pcl::search::Octree<PointT> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);
    tracker_->setCloudCoherence (coherence);

}

void track(pcl::PointCloud<PointT>::Ptr cloud_input){
    tracker_->setInputCloud(cloud_input);
    tracker_->setReferenceCloud(object_to_track);
    tracker_->compute();
    pcl::tracking::ParticleXYZRPY result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
    pcl::transformPointCloud<PointT>(*object_to_track,*tracked_object,transformation);
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
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
            if(index_to_track+1 < object_vector.size()){
                index_to_track++;
            }
            else{
                index_to_track = 0;
            }
            object_to_track = object_vector[index_to_track];
            tracker_->resetTracking();
        }
    }
    else{
        l_count = 0;
    }

}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segment_objects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, double tolerance, int minClusterSize, int maxClusterSize){
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (tolerance); // 2cm
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_input);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_input);
    ec.extract (cluster_indices);

    // returns a vector of all the objects
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_vector_temp;
    for (int i=0; i<cluster_indices.size(); i++){
        pcl::PointIndices cloud_indices = cluster_indices.at(i);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_cluster = extract_object_from_indices(cloud_input,cloud_indices);
        object_vector_temp.push_back(cloud_cluster);
    }

    return object_vector_temp;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_object_from_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,pcl::PointIndices object_indices){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int j=0; j<object_indices.indices.size(); j++){
        cloud_cluster->points.push_back (cloud_input->points[object_indices.indices[j]]);
    }
    return cloud_cluster;
}

// Compute centroid of an object
Eigen::Matrix<float,4,1> compute_centroid_point(const pcl::PointCloud<PointT>& p_point_cloud)
{
    pcl::ConstCloudIterator<PointT> it(p_point_cloud);
    Eigen::Matrix< float, 4, 1 > matrix;
    pcl::compute3DCentroid(it, matrix);
    return matrix;
}


// Compute the distance between the camera and the centroid
float compute_distance_from_kinect(Eigen::Matrix<float, 4, 1> p_matrix)
{

    pcl::PointXYZ camera_origin(0,0,0);
    pcl::PointXYZ object_position(p_matrix(0,0), p_matrix(1,0), p_matrix(3,0));
    float distance = pcl::euclideanDistance(camera_origin, object_position);

    return distance;
}

//find the point cloud limit and put it in CORNER_CLOUD global variable
void point_cloud_limit_finder (Eigen::Matrix<float, 4, 1> p_matrix, pcl::PointCloud<PointT>::Ptr p_ptr)
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

    find_corner(left,right,top,bottom,POINT_CLOUD_CORNER);
}

//find the square corner for an object
void find_corner(const PointT& p_left, const PointT& p_right, const PointT& p_top, const PointT& p_bottom, pcl::PointCloud<PointT>::Ptr& p_point_cloud_output)
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



//project from 3d to 2d in pixel coordinate a point from a martix
Eigen::Matrix<float,4,1> projection2d_matrix(const Eigen::Matrix<float,4,1>& p_matrix)
{
    Eigen::Matrix<float,4,1> matrix_2d;
    matrix_2d(0,0) = (((p_matrix(0,0) * 525)/p_matrix(2,0))+320);
    matrix_2d(1,0) = (((p_matrix(1,0) * 525)/p_matrix(2,0))+240);
    matrix_2d(2,0) = p_matrix(2,0)/p_matrix(2,0);
    matrix_2d(3,0) = p_matrix(3,0);
    return matrix_2d;
}

//project from 3d to 2d in pixel coordinate a point cloud put it in the global variable OBJECT_VECTOR_2D
void projection2d_pointCloud(const pcl::PointCloud<PointT>& p_point_cloud, std::vector<pcl::PointCloud<PointT>::Ptr>& p_vector_output)
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

//modifie the pixel at the given coordinate
void change_pixel_color(std::vector<unsigned char>& p_array, int p_x, int p_y, int p_b = 255, int p_g = 0, int p_r = 0)
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


//draw the square into the rgb image
void draw_square(std::vector<unsigned char>& p_array, PointT p_top_left, PointT p_top_right, PointT p_bottom_left, PointT p_bottom_right)
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

void image_processing(pcl::PointCloud<PointT>::Ptr p_point_cloud_corner, sensor_msgs::Image p_image_input)
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
    IMAGE_MEMORY.header = p_image_input.header;
    IMAGE_MEMORY.encoding = p_image_input.encoding;
    IMAGE_MEMORY.width = p_image_input.width;
    IMAGE_MEMORY.height = p_image_input.height;
    IMAGE_MEMORY.step = p_image_input.step;
    for(int  i =0; i < p_image_input.data.size(); i ++)
    {
        IMAGE_MEMORY.data.push_back(p_image_input.data.at(i));
    }
    PUB2.publish(p_image_input);
}

int position_finder_vector(const std_msgs::UInt32MultiArray& p_coordinate, const pcl::PointCloud<PointT>& p_point_cloud_corner)
{
    for(int i = 0; i < p_point_cloud_corner.size(); i += 4)
    {
        PointT point_temps_top_left = p_point_cloud_corner.at(i);
        PointT point_temps_bottom_right = p_point_cloud_corner.at(i+1);
        if(p_coordinate.data.at(0) >= point_temps_top_left.x and p_coordinate.data.at(0) <= point_temps_bottom_right.y)
        {
            if(p_coordinate.data.at(1) <= point_temps_top_left.x and p_coordinate.data.at(1) >= point_temps_bottom_right.y)
                return i/4;
        }
    }
    return -1;
}

// Callback Function for the subscribed ROS topic
void cloud_callback (const pcl::PCLPointCloud2ConstPtr& input){

    POINT_CLOUD_RECEIVED = true;

    boost::unique_lock<boost::mutex> scoped_lock(MTX);


    CORNER_CLOUD.clear();
    OBJECT_VECTOR_2D.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*input,*objects);
    cloud = objects;

    //Extract objects
    if(!object_vector.empty()){object_vector.clear();}
    object_vector = segment_objects(objects,0.02,200,8000);

/*
    if(initialize_object_to_track){
        object_to_track = object_vector[index_to_track];
        object_to_track->height = 1;
        object_to_track->width = object_to_track->size();
        initialize_object_to_track = false;
    }
    else{
        track(objects);
    }
*/
    //Update viewer
    if(showUI){
        printToPCLViewer();
    }

    //project all the objcect in 3d to a 2d plan
    for(int i = 0; i < object_vector.size(); i++)
    {
        projection2d_pointCloud(*(object_vector.at(i)), OBJECT_VECTOR_2D);
    }

    //calculate the distance for each objects in the vector
    POINT_CLOUD_CORNER->clear();
    for(int i =0; i < object_vector.size(); i++)
    {

        Eigen::Matrix<float,4,1> matrix = compute_centroid_point(*(object_vector.at(i)));
        compute_distance_from_kinect(matrix);
        Eigen::Matrix<float,4,1> matrix_2d = projection2d_matrix(matrix);
        point_cloud_limit_finder(matrix_2d, OBJECT_VECTOR_2D.at(i));

    }

}

void callback_function2(const sensor_msgs::Image& p_input)
{

    boost::unique_lock<boost::mutex> scoped_lock(MTX);

    IMAGE_RECEIVED_INPUT = p_input;
    IMAGE_RECEIVED_INPUT.header = p_input.header;

}

void callback_function3(const std_msgs::UInt32MultiArray& p_input)
{
    COORDINATE_USER_SENDED = p_input;
    COORDINATE_RECEIVED = true;

}



int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "object_extraction");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = n.subscribe ("/custom/not_planes", 1, cloud_callback);
    ros::Subscriber sub2 = n.subscribe("/camera/rgb/image_color", 1, callback_function2);
    ros::Subscriber sub3 = n.subscribe("/image_coordinate_rgb", 1, callback_function3);

    // Create a ROS publisher for the output point cloud
    //pub = n.advertise<sensor_msgs::PointCloud2> ("/extracted_planes", 1);
    //publish the objects limits
    pub = n.advertise<pcl::PointCloud<PointT> > ("/corner_limits", 1);//the one that I use right now
    PUB2 = n.advertise<sensor_msgs::Image>("/square_image", 1);

    // Load parameters from launch file
    nh.param("objects_visualizer",showUI,true);
    //showUI = true;

    // Initialize tracking system
    initTracking();

    //PCL Viewer
    bool loop_condition = true;
    if(showUI){
        pclViewer->setBackgroundColor (0, 0, 0);
        pclViewer->initCameraParameters ();
        pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(800,450);
        renderWindow->Render();
        pclViewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&pclViewer);
    }
    else{ // Not very clean, but only solution I found to not show pclViewer
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(1,1);
        renderWindow->Render();
    }

    ros::Rate r(5);
    while (loop_condition) {
        ros::spinOnce();
        pclViewer->spinOnce (100);
        if(COORDINATE_RECEIVED)
        {
            //coordinate traitement on the memory cloud + memory image
            int position_in_vector = position_finder_vector(COORDINATE_USER_SENDED, *MEMORY_POINT_CLOUD_CORNER_PTR);
        }
        if(POINT_CLOUD_RECEIVED)
        {
            image_processing(POINT_CLOUD_CORNER, IMAGE_RECEIVED_INPUT);
            *MEMORY_POINT_CLOUD_CORNER_PTR = *POINT_CLOUD_CORNER;
            MEMORY_POINT_CLOUD = object_vector; // verify what are the best cloud to keep in memory
        }
        else
        {
            PUB2.publish(IMAGE_MEMORY);
        }

        POINT_CLOUD_RECEIVED = false;
        COORDINATE_RECEIVED = false;
        loop_condition = ros::ok() && !pclViewer->wasStopped();
        r.sleep();
    }
    return 0;
}

