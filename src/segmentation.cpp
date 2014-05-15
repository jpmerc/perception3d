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
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>), cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

// PCL Viewer
bool showUI = true;
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

pcl::PCLPointCloud2Ptr voxelgrid_filter(const pcl::PCLPointCloud2ConstPtr cloud_input, float leaf_size){
    pcl::VoxelGrid<pcl::PCLPointCloud2> vx_grid;
    vx_grid.setInputCloud (cloud_input);
    vx_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    vx_grid.filter (*cloud_filtered);
    return cloud_filtered;
}

pcl::PCLPointCloud2Ptr passthrough_filter(pcl::PCLPointCloud2Ptr cloud_input, double min_distance, double max_distance){
    pcl::PassThrough<pcl::PCLPointCloud2> pt_filter;
    pt_filter.setFilterFieldName ("z");
    pt_filter.setFilterLimits (min_distance, max_distance);
    pt_filter.setKeepOrganized (false);
    pt_filter.setInputCloud (cloud_input);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    pt_filter.filter (*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_outlier_removal_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, double radius, int minNN){
    //std::cout << "#of points before : " << cloud_input->size() << std::endl;

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(cloud_input);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius (minNN);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    outrem.filter (*cloud_filtered);
   // std::cout << "#of points after : " << cloud_filtered->size() << std::endl;
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, int maxNumberOfPlanesToExtract){
    // Segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Instances used in planes extraction
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_seg_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_planes(new pcl::PointCloud<pcl::PointXYZRGB>);

    int iterationNumber = 1;
    while(iterationNumber <= maxNumberOfPlanesToExtract){
        seg.setInputCloud (cloud_input);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_pointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            return empty_pointcloud; //Returns empty pointcloud
        }

        // If the other planes (other than the principal) correspond to less than 20% of the pointcloud, they are not taken
        double pointcloud_proportion = double(inliers->indices.size())/double(cloud_input->size());
        if(iterationNumber >= 2 && pointcloud_proportion < 0.2){
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_input);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*temp_seg_cloud);
        *segmented_planes += *temp_seg_cloud;

        extract.setNegative (true);
        extract.filter (*temp_seg_cloud);
        cloud_input.swap (temp_seg_cloud);

        iterationNumber++;
    }

    return segmented_planes;
}


void euclidean_object_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,std::vector<pcl::PointIndices> &cluster_indices){
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (500);
    ec.setMaxClusterSize (15000);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_input);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_input);
    ec.extract (cluster_indices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_object_from_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,pcl::PointIndices object_indices){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int j=0; j<object_indices.indices.size(); j++){
        cloud_cluster->points.push_back (cloud_input->points[object_indices.indices[j]]);
    }
    return cloud_cluster;
}


// Callback Function for the subscribed ROS topic
void cloud_callback (const pcl::PCLPointCloud2ConstPtr& input){
    //pcl::fromPCLPointCloud2(*input,*cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
    pcl::PCLPointCloud2Ptr cloud_filtered = voxelgrid_filter(input, 0.005f);

    // Passthrough filter
    cloud_filtered = passthrough_filter(cloud_filtered,0,2.5);

    // Transform pc2 to pc
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_filtered,*cloud_filtered2);

    // To show the filtered data in the pointcloud viewer
    *cloud = *cloud_filtered2;

    // Plane segmentation
    segmented_cloud = plane_segmentation(cloud_filtered2,3);
    if(segmented_cloud->size() == 0) return;
    // std::cout << "PointCloud representing the planar components: " << segmented_cloud->width * segmented_cloud->height << " data points." << std::endl;

    cloud_filtered2 = radius_outlier_removal_filter(cloud_filtered2,0.05,30);



    //   ==============  Euclidean Object Clustering  ==============  //
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php

    //    // Here, the extracted planes are not included in cloud_filtered2 (extracted by plane_segmentation function)
    //    std::vector<pcl::PointIndices> cluster_indices;
    //    euclidean_object_segmentation(cloud_filtered2,cluster_indices);
    //    std::cout << "Number of found objects: " << cluster_indices.size() << std::endl;

    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
    //    for (int i=0; i<cluster_indices.size(); i++){
    //        pcl::PointIndices cloud_indices = cluster_indices.at(i);
    //        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    //        cloud_cluster = extract_object_from_indices(cloud_filtered2,cloud_indices);
    //        *object_clusters += *cloud_cluster;
    //    }
    //    objects_cloud.swap(object_clusters);

    //   ===========================================================  //

    // pc to pc2
    pcl::PCLPointCloud2 planes_pcl,not_planes_pcl;
    pcl::toPCLPointCloud2(*segmented_cloud,planes_pcl);
    pcl::toPCLPointCloud2(*cloud_filtered2,not_planes_pcl);

    //From PCL pointclouds to ROS pointclouds (sensor_msgs)
    sensor_msgs::PointCloud2 filtered, planes,not_planes;
    pcl_conversions::fromPCL(planes_pcl,planes);
    pcl_conversions::fromPCL(*cloud_filtered,filtered);
    pcl_conversions::fromPCL(not_planes_pcl,not_planes);

    pcl_conversions::fromPCL(input->header,planes.header);
    pcl_conversions::fromPCL(input->header,filtered.header);
    pcl_conversions::fromPCL(input->header,not_planes.header);

    // Publish on given topics
    pub.publish (planes);
    pub2.publish(filtered);
    pub3.publish (not_planes);

    //Update PCL Viewer
    if(showUI){
        printToPCLViewer();
    }

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "segmentation");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = n.subscribe ("/camera/depth_registered/points", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2> ("/custom/planes", 1);
    pub2 = n.advertise<sensor_msgs::PointCloud2> ("/custom/filtered_data", 1);
    pub3 = n.advertise<sensor_msgs::PointCloud2> ("/custom/not_planes", 1);

    // Load parameters from launch file
    nh.param("pcl_visualizer",showUI,true);


    //PCL Viewer
    bool loop_condition = true;
    if(showUI){
        pclViewer->setBackgroundColor (0, 0, 0);
        pclViewer->initCameraParameters ();
        pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(800,450);
        renderWindow->Render();
    }
    else{ // Not very clean, but only solution I found to not show pclViewer
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(1,1);
        renderWindow->Render();
    }

    ros::Rate r(30);
    while (loop_condition) {
        ros::spinOnce();
        if(showUI){
            pclViewer->spinOnce (100);
            loop_condition = ros::ok() && !pclViewer->wasStopped();
        }
        else{
            loop_condition = ros::ok();
        }
        r.sleep();
    }
    return 0;
}
