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

#include <stdio.h>
#include <stdlib.h>



typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_vector;

// PCL Viewer
bool showUI = true;
int l_count = 0;
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer 2"));

//Function declarations
pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_object_from_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,pcl::PointIndices object_indices);


void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    //    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"source cloud");
    //    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

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
    pcl::PointXYZ camera_origine(0,0,0);
    pcl::PointXYZ object_position(p_matrix(0,0), p_matrix(1,0), p_matrix(2,0));
    float distance = pcl::euclideanDistance(camera_origine, object_position);
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

    //Update viewer
    if(showUI){
        printToPCLViewer();
    }

    //calculate the distance for each objects in the vector
    for(int i =0; i < object_vector.size(); i++)
    {
    Eigen::Matrix<float,4,1> matrix = compute_centroid_point(*(object_vector.at(i)));
    compute_distance_from_kinect(matrix);
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
