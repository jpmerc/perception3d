#include <ros/ros.h>

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
#include <pcl/visualization/keyboard_event.h>

#include <vtkRenderWindow.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>

#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt32MultiArray.h>
#include <boost/thread/mutex.hpp>
#include <stdio.h>
#include <stdlib.h>

class ObjectExtractor{
public:
    typedef pcl::PointXYZRGB PointT;

    //methods
    ObjectExtractor(bool showViewer);
    void extraction_callback(const pcl::PCLPointCloud2ConstPtr& input);
    std::vector<pcl::PointCloud<PointT>::Ptr> segment_objects(pcl::PointCloud<PointT>::Ptr cloud_input, double tolerance, int minClusterSize, int maxClusterSize);
    pcl::PointCloud<PointT>::Ptr extract_object_from_indices(pcl::PointCloud<PointT>::Ptr cloud_input,pcl::PointIndices object_indices);
    bool* get_showUI();
    void set_showUI(bool show);

    //variables

private:

    //variables
    pcl::PointCloud<PointT>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer;
    std::vector<pcl::PointCloud<PointT>::Ptr> object_vector;
    pcl::PointCloud<PointT>::Ptr object_to_track;
    int index_to_track;
    bool showUI;
    int l_count;



};
