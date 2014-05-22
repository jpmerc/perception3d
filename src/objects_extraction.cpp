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
#include <stdio.h>
#include <stdlib.h>
#include <pcl/cloud_iterator.h>
#include<pcl/common/centroid.h>
#include<pcl/common/distances.h>

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
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr object_to_track(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr tracked_object(new pcl::PointCloud<PointT>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_vector;
boost::shared_ptr<ParticleFilter> tracker_(new ParticleFilter);
int index_to_track = 0;

// PCL Viewer
bool showUI = true;
int l_count = 0;
bool initialize_object_to_track = true;
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer 2"));

//Function declarations
pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_object_from_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,pcl::PointIndices object_indices);

void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"source cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
//    for(int i=0; i < object_vector.size(); i++){
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = object_vector[i];
//        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> randColor(pc);
//        std::stringstream ss;
//        ss << i;
//        std::string ind = ss.str();
//        std::string pc_name = "object_" + ind;
//        pclViewer->addPointCloud<pcl::PointXYZRGB>(pc,randColor,pc_name);
//        pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pc_name);
//    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> yellow_color(tracked_object, 255, 255, 102);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(tracked_object,yellow_color,"tracked cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tracked cloud");
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

// Callback Function for the subscribed ROS topic
void cloud_callback (const pcl::PCLPointCloud2ConstPtr& input){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*input,*objects);
    cloud = objects;

    //Extract objects
    if(!object_vector.empty()){object_vector.clear();}
    object_vector = segment_objects(objects,0.02,200,15000);

    if(initialize_object_to_track){
        object_to_track = object_vector[index_to_track];
        object_to_track->height = 1;
        object_to_track->width = object_to_track->size();
        initialize_object_to_track = false;
    }
    else{
        track(objects);
    }

    //Update viewer
    if(showUI){
        printToPCLViewer();
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "object_extraction");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = n.subscribe ("/custom/not_planes", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    //pub = n.advertise<sensor_msgs::PointCloud2> ("/extracted_planes", 1);


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

    ros::Rate r(30);
    while (loop_condition) {
        ros::spinOnce();
        pclViewer->spinOnce (100);
        loop_condition = ros::ok() && !pclViewer->wasStopped();
        r.sleep();
    }
    return 0;
}
