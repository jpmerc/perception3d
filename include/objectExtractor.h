#ifndef objectExtractor_H
#define objectExtractor_H

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
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
#include <stdio.h>
#include <stdlib.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include<object_recognition.h>


class ObjectExtractor{
public:
    typedef pcl::PointXYZRGB PointT;

    //Methods
    ObjectExtractor(bool showViewer, ros::NodeHandle p_nh);
    void extraction_callback(const pcl::PCLPointCloud2ConstPtr& input);
    bool get_showUI();
    void set_showUI(bool show);
    void toggle_showUI();
    Eigen::Vector4f getGraspCentroid();
    pcl::PointCloud<PointT>::Ptr getObjectToGrasp();
    tf::StampedTransform getCentroidPositionRGBFrame();

    void callback_rgb_camera(const sensor_msgs::Image& p_input);
    Eigen::Matrix<float,4,1> compute_centroid_point(const pcl::PointCloud<PointT>& p_point_cloud);
    float compute_distance_from_kinect(Eigen::Matrix<float, 4, 1> p_matrix);
    void point_cloud_limit_finder (Eigen::Matrix<float, 4, 1> p_matrix, pcl::PointCloud<PointT>::Ptr p_ptr);
    void find_corner(const PointT& p_left, const PointT& p_right, const PointT& p_top, const PointT& p_bottom, pcl::PointCloud<PointT>::Ptr& p_point_cloud_output);
    Eigen::Matrix<float,4,1> projection2d_matrix(const Eigen::Matrix<float,4,1>& p_matrix);
    void projection2d_pointCloud(const pcl::PointCloud<PointT>& p_point_cloud, std::vector<pcl::PointCloud<PointT>::Ptr>& p_vector_output);
    void change_pixel_color(std::vector<unsigned char>& p_array, int p_x, int p_y, int p_b = 255, int p_g = 0, int p_r = 0);
    void draw_square(std::vector<unsigned char>& p_array, PointT p_top_left, PointT p_top_right, PointT p_bottom_left, PointT p_bottom_right);
    void image_processing(pcl::PointCloud<PointT>::Ptr p_point_cloud_corner, sensor_msgs::Image p_image_input);
    int position_finder_vector(const float p_coordinate[], const pcl::PointCloud<PointT>& p_point_cloud_corner, const std::vector<float> p_distance_vector);

    int coordinate_processing(const float p_coordinate[],
                               pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd);
    void point_cloud_processing();

    void spin_once();


    // Variables
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer;
    Object_recognition m_object_recognition;

private:
    // Methods
    void printToPCLViewer();
    void keyboard_callback (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
    std::vector<pcl::PointCloud<PointT>::Ptr> segment_objects(pcl::PointCloud<PointT>::Ptr cloud_input, double tolerance, int minClusterSize, int maxClusterSize);
    pcl::PointCloud<PointT>::Ptr extract_object_from_indices(pcl::PointCloud<PointT>::Ptr cloud_input,pcl::PointIndices object_indices);
    void setPCLViewer();
    void refreshObjectCentroid();



    // Variables
    pcl::PointCloud<PointT>::Ptr cloud;
    std::vector<pcl::PointCloud<PointT>::Ptr> object_vector;
    pcl::PointCloud<PointT>::Ptr object_to_grasp;
    pcl::PointCloud<PointT>::Ptr tracked_object_centroid;
    int index_to_grasp;
    bool initialize_object_to_grasp;
    bool showUI;
    int l_count;

    ros::Publisher m_pub_image;
    ros::Publisher m_pub_android;

    std::vector<pcl::PointCloud<PointT>::Ptr> m_object_vector_2d;
    pcl::PointCloud<PointT> m_corner_cloud;
    pcl::PointCloud<PointT>::Ptr m_point_cloud_corner_ptr;

    std::vector<pcl::PointCloud<PointT>::Ptr> m_memory_point_cloud;
    pcl::PointCloud<PointT>::Ptr m_memory_point_cloud_corner_ptr;
    bool m_point_cloud_received;
    bool m_coordinate_received;
    float m_coordinate_user_sended[2];
    std::vector<float> m_memory_distance_vector;
    std::vector<float> m_distance_vector;
    sensor_msgs::Image m_image_received_input;
    sensor_msgs::Image m_image_memory;



    int NumberOfSnapshots;
    std::string directory;

};

#endif
