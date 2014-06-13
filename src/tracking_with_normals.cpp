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

#include<pcl/segmentation/extract_polygonal_prism_data.h>
#include<pcl/surface/convex_hull.h>
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

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/tracking/impl/kld_adaptive_particle_filter_omp.hpp>

#define PCL_TRACKING_NORMAL_SUPPORTED true




typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointTNormal;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointTNormal, ParticleT> ParticleFilter;

ros::Publisher pub;
ros::Publisher PUB2;
pcl::PointCloud<PointTNormal>::Ptr cloud(new pcl::PointCloud<PointTNormal>);
pcl::PointCloud<PointTNormal>::Ptr object_to_track(new pcl::PointCloud<PointTNormal>);
pcl::PointCloud<PointTNormal>::Ptr tracked_object(new pcl::PointCloud<PointTNormal>);
boost::shared_ptr<ParticleFilter> tracker_(new ParticleFilter);
std::vector<pcl::PointCloud<PointT>::Ptr> object_vector;
int index_to_track = 0;


pcl::PointCloud<PointT>::Ptr depth_registered_cloud(new pcl::PointCloud<PointT>);
boost::mutex depth_cloud_mutex;
ParticleFilter::PointCloudStatePtr particles;
bool first_time_tracking = true;


// PCL Viewer
bool showUI = true;
int l_count = 0;
bool initialize_object_to_track = true;
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer 2"));

//Function declarations
pcl::PointCloud<PointT>::Ptr extract_object_from_indices(pcl::PointCloud<PointT>::Ptr cloud_input,pcl::PointIndices object_indices);
pcl::PointCloud<PointTNormal>::Ptr addNormalsToPointCloud(pcl::PointCloud<PointT>::Ptr cloud_input);
void setTrackerTarget();
void track();
Eigen::Matrix<float,4,1> compute_centroid_point(const pcl::PointCloud<PointT>& p_point_cloud);

void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    //pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    //pclViewer->addPointCloud<PointT>(cloud,rgb,"source cloud");
    //pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
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

    pcl::visualization::PointCloudColorHandlerCustom<PointTNormal> yellow_color(tracked_object, 255, 255, 102);
    pclViewer->addPointCloud<PointTNormal>(tracked_object,yellow_color,"tracked cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tracked cloud");


//    if(particles){
//        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//        for (size_t i = 0; i < particles->points.size (); i++){
//            pcl::PointXYZ point;
//            point.x = particles->points[i].x;
//            point.y = particles->points[i].y;
//            point.z = particles->points[i].z;
//            particle_cloud->points.push_back(point);
//        }
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);
//        pclViewer->addPointCloud<pcl::PointXYZ> (particle_cloud, red_color, "particle cloud");
//        pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "particle cloud");

//    }
}

void initTracking(){
    // based on PCL apps : https://github.com/PointCloudLibrary/pcl/blob/master/apps/src/openni_tracking.cpp

    //Particle filter with 8 threads
    boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointTNormal, ParticleT> > tracker(new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointTNormal, ParticleT> (8));

    //TODO check the meaning of parameters
    tracker->setMaximumParticleNum (1000);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    //tracker_->setAlpha(1);
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
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.001);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    tracker_->setTrans (Eigen::Affine3f::Identity ());
    //tracker_->setMotionRatio(0.5);
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);
    tracker_->setMinIndices (100);
    //tracker->setUseChangeDetector(false);


    //Setup Coherence
    pcl::tracking::ApproxNearestPairPointCloudCoherence<PointTNormal>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<PointTNormal>::Ptr(new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointTNormal> ());
    boost::shared_ptr<pcl::tracking::DistanceCoherence<PointTNormal> > distance_coherence = boost::shared_ptr<pcl::tracking::DistanceCoherence<PointTNormal> > (new pcl::tracking::DistanceCoherence<PointTNormal> ());
    boost::shared_ptr<pcl::tracking::HSVColorCoherence<PointTNormal> > color_coherence = boost::shared_ptr<pcl::tracking::HSVColorCoherence<PointTNormal> > (new pcl::tracking::HSVColorCoherence<PointTNormal> ());
    boost::shared_ptr<pcl::tracking::NormalCoherence<PointTNormal> > normal_coherence = boost::shared_ptr<pcl::tracking::NormalCoherence<PointTNormal> > (new pcl::tracking::NormalCoherence<PointTNormal> ());
    distance_coherence->setWeight(0.5);
    color_coherence->setWeight (0.2);
    normal_coherence->setWeight (0.5);
    coherence->addPointCoherence (distance_coherence);
    coherence->addPointCoherence (color_coherence);
    coherence->addPointCoherence (normal_coherence);
    boost::shared_ptr<pcl::search::Octree<PointTNormal> > search (new pcl::search::Octree<PointTNormal> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.10);
    tracker_->setCloudCoherence (coherence);

}

void setTrackerTarget(){
    initTracking();
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    pcl::compute3DCentroid<PointTNormal>(*object_to_track,c);
    trans.translation().matrix() = Eigen::Vector3f(c[0],c[1],c[2]);
    tracker_->setTrans (trans);

    pcl::PointCloud<PointTNormal>::Ptr tmp_pc(new pcl::PointCloud<PointTNormal>);
    pcl::transformPointCloud<PointTNormal> (*object_to_track, *tmp_pc, trans.inverse());

    tracker_->setReferenceCloud(tmp_pc);
    tracker_->setInputCloud(cloud);

}


void track(){
    //pcl::PointCloud<PointTNormal>::Ptr cloudWithNormals = addNormalsToPointCloud(cloud_input);
      tracker_->setInputCloud(cloud);


    //   tracker_->setReferenceCloud(object_to_track);
    //    tracker_->getCloudCoherence()->setTargetCloud(cloud);
    //    const pcl::PointCloud<PointT>::ConstPtr tmp = tracker_->getInputCloud();
    //    const pcl::PointCloud<PointT>::ConstPtr tmp2 = tracker_->getReferenceCloud();
    tracker_->compute();
    pcl::tracking::ParticleXYZRPY result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
    //transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
    Eigen::Affine3f transs = tracker_->getTrans();
    pcl::transformPointCloudWithNormals<PointTNormal>(*(tracker_->getReferenceCloud ()),*tracked_object,transformation);

    particles = tracker_->getParticles ();
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
             object_to_track = addNormalsToPointCloud(object_vector[index_to_track]);
            //object_to_track = object_vector[index_to_track];
            object_to_track->height = 1;
            object_to_track->width = object_to_track->size();
            setTrackerTarget();
        }
    }
    else{
        l_count = 0;
    }

}

std::vector<pcl::PointCloud<PointT>::Ptr> segment_objects(pcl::PointCloud<PointT>::Ptr cloud_input, double tolerance, int minClusterSize, int maxClusterSize){
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

pcl::PointCloud<PointTNormal>::Ptr addNormalsToPointCloud(pcl::PointCloud<PointT>::Ptr cloud_input){
    pcl::PointCloud<PointTNormal>::Ptr cloudWithNormals(new pcl::PointCloud<PointTNormal>());

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud (cloud_input);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);
    pcl::concatenateFields(*cloud_input,*cloud_normals,*cloudWithNormals);

    return cloudWithNormals;
}

pcl::PointCloud<PointT>::Ptr extract_object_from_indices(pcl::PointCloud<PointT>::Ptr cloud_input,pcl::PointIndices object_indices){
    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
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

    pcl::PointCloud<PointT>::Ptr objects(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*input,*objects);
    cloud = addNormalsToPointCloud(objects);

    //Extract objects
    if(!object_vector.empty()){object_vector.clear();}
    object_vector = segment_objects(objects,0.02,200,15000);


    if(initialize_object_to_track){
        object_to_track = addNormalsToPointCloud(object_vector[index_to_track]);
       // object_to_track = object_vector[index_to_track];
        object_to_track->height = 1;
        object_to_track->width = object_to_track->size();
        initialize_object_to_track = false;
    }

    // depth_cloud_mutex.lock();
    if(first_time_tracking){
        setTrackerTarget();
        first_time_tracking = false;
    }
    track();
    // depth_cloud_mutex.unlock();


    //Update viewer
    if(showUI){
        printToPCLViewer();
    }
}

void depth_registered_callback(const pcl::PCLPointCloud2ConstPtr& input){
    depth_cloud_mutex.lock();
    pcl::fromPCLPointCloud2(*input,*depth_registered_cloud);
    depth_cloud_mutex.unlock();
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "tracking");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = n.subscribe ("/custom/not_planes", 1, cloud_callback);
    ros::Subscriber sub2 = n.subscribe ("/camera/depth_registered/points", 1, depth_registered_callback);

    // Load parameters from launch file
    nh.param("objects_visualizer",showUI,true);
    //showUI = true;

    // Initialize tracking system
    initTracking();

    //PCL Viewer
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
    while (ros::ok() && !pclViewer->wasStopped()) {
        ros::spinOnce();
        pclViewer->spinOnce (100);
        r.sleep();
    }


    return 0;
}


