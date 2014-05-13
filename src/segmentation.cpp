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
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>



ros::Publisher pub;
ros::Publisher pub2;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>), cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

// PCL Viewer
bool refreshUI = true;
int l_count = 0;
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"source cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(segmented_cloud, 255, 0, 0);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(segmented_cloud,red_color,"segmented cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmented cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color(objects_cloud, 0, 0, 255);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(objects_cloud,blue_color,"objects cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "objects cloud");

}


// Callback Function for the subscribed ROS topic
void cloud_cb (const pcl::PCLPointCloud2ConstPtr& input){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud_local(new pcl::PointCloud<pcl::PointXYZRGB>);

    //pcl::fromPCLPointCloud2(*input,*cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (input);
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    pcl::PCLPointCloud2 cloud_filtered;
    sor.filter (cloud_filtered);

    // Transform pc2 to pc
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud_filtered,*cloud_filtered2);

    // To show the filtered data in the pointcloud viewer
    *cloud = *cloud_filtered2;

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


    // Instances used in planes extraction
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_seg_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    int iterationNumber = 1;
    int maxNumberOfPlanesToExtract = 3;
    while(iterationNumber <= maxNumberOfPlanesToExtract){
        seg.setInputCloud (cloud_filtered2);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            return;
        }

        // If the other planes (other than the principal) correspond to less than 25% of the pointcloud, they are not taken
        double pointcloud_proportion = double(inliers->indices.size())/double(cloud_filtered2->size());
        if(iterationNumber >= 2 && pointcloud_proportion < 0.15){
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered2);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*temp_seg_cloud);
        *segmented_cloud_local += *temp_seg_cloud;

        extract.setNegative (true);
        extract.filter (*temp_seg_cloud);
        cloud_filtered2.swap (temp_seg_cloud);

        iterationNumber++;
    }
    segmented_cloud.swap(segmented_cloud_local);
    std::cout << "PointCloud representing the planar components: " << segmented_cloud->width * segmented_cloud->height << " data points." << std::endl;



    //   ==============  Euclidean Object Clustering  ==============  //
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (500);
    ec.setMaxClusterSize (15000);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered2);
    ec.setSearchMethod (tree);

    ec.setInputCloud (cloud_filtered2);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cloud_cluster->points.push_back (cloud_filtered2->points[*pit]);
        }
        *object_clusters += *cloud_cluster;
    }
    objects_cloud.swap(object_clusters);


    // Publish the pointclouds
    pcl::PCLPointCloud2 segmented_pcl;
    pcl::toPCLPointCloud2(*segmented_cloud,segmented_pcl);
    sensor_msgs::PointCloud2 segmented, not_segmented;
    pcl_conversions::fromPCL(cloud_filtered,segmented);
    pcl_conversions::fromPCL(*input,not_segmented);
    pub.publish (segmented);
    pub2.publish(not_segmented);

    //Update PCL Viewer
    printToPCLViewer();

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "segmentation");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/extracted_planes", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/extracted_planes_not", 1);


    //PCL Viewer
    pclViewer->setBackgroundColor (0, 0, 0);
    pclViewer->initCameraParameters ();
    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(800,450);
    renderWindow->Render();

    ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped()) {
        pclViewer->spinOnce (100);
        ros::spinOnce();
        // boost::this_thread::sleep (boost::posix_time::microseconds (10000));
        r.sleep();
    }
    return 0;
}
