#include <ros/ros.h>
// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <angles/angles.h>

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
pcl::PointCloud<PointT>::Ptr segmented_cloud(new pcl::PointCloud<PointT>), cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr objects_cloud(new pcl::PointCloud<PointT>);
std::vector<pcl::PointCloud<PointT>::Ptr> object_vector;

// PCL Viewer
bool showUI = true;
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
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
}

pcl::PointCloud<PointT>::Ptr connected_segmentation(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<pcl::Normal>::Ptr normals){
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;

    int numberOfinliers = 5000;

    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers (5000);
    mps.setAngularThreshold (pcl::deg2rad (2.0)); //2 degrees
    mps.setDistanceThreshold (0.02); //2cm
    mps.setInputNormals (normals);
    mps.setInputCloud (cloud_input);
    mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);


    //Object clustering
    std::vector<bool> plane_labels;
    plane_labels.resize (label_indices.size (), false);
    for (int i = 0; i < label_indices.size (); i++)
    {
        if (label_indices[i].indices.size () > numberOfinliers)
        {
            plane_labels[i] = true;
        }
    }

    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
    euclidean_cluster_comparator_->setInputCloud (cloud_input);
    euclidean_cluster_comparator_->setLabels (labels);
    euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
    euclidean_cluster_comparator_->setDistanceThreshold (0.02f, true);

    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
    euclidean_segmentation.setInputCloud (cloud_input);
    euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

    object_vector.clear();

    //std::cout << "# d'objets = " << euclidean_label_indices.size () << std::endl;
    for (int i = 0; i < euclidean_label_indices.size (); i++)
    {
        if (euclidean_label_indices[i].indices.size () > 1000)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,*cluster);
            object_vector.push_back (cluster);
        }
    }

}

pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(pcl::PointCloud<PointT>::Ptr cloud_input){
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX); //Changing the method can make it faster
    ne.setMaxDepthChangeFactor (0.01f);
    ne.setNormalSmoothingSize (20.0f);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud (cloud_input);
    ne.compute (*normal_cloud);
    return normal_cloud;
}


// Callback Function for the subscribed ROS topic
void cloud_callback (const pcl::PCLPointCloud2ConstPtr& input){

    // Transform pc2 to pc
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*input,*cloud_filtered2);

    // To show the filtered data in the pointcloud viewer
    *cloud = *cloud_filtered2;

    // =======================Plane segmentation===========================
    // Estimate Normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud = estimate_normals(cloud_filtered2);

    // Segment
    connected_segmentation(cloud_filtered2,normal_cloud);

    //Update PCL Viewer
    printToPCLViewer();


}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "segmentation");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = n.subscribe ("/camera/depth_registered/points", 1, cloud_callback);

    //PCL Viewer
    pclViewer->setBackgroundColor (0, 0, 0);
    pclViewer->initCameraParameters ();
    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(800,450);
    renderWindow->Render();
    ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped()) {
        ros::spinOnce();
        pclViewer->spinOnce (100);
        r.sleep();
    }
    return 0;
}
