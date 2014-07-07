#ifndef planSegmentor_H
#define planSegmentor_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCPointT;


class planSegmentor
{
public:

    planSegmentor(ros::NodeHandle p_nh);

    void cloud_callback(const pcl::PCLPointCloud2ConstPtr& p_input);

    pcl::PCLPointCloud2Ptr voxelgrid_filter(const pcl::PCLPointCloud2ConstPtr p_cloud,
                                            float p_leaf_size);

    pcl::PCLPointCloud2Ptr passthrough_filter(pcl::PCLPointCloud2Ptr p_input,
                                               double p_min_distance,
                                               double p_max_distance);

    PCPointT::Ptr plane_segmentation(PCPointT::Ptr p_cloud,
                                     int p_maxNumberOfPlanesToExtract);

    PCPointT::Ptr radius_outlier_removal_filter(PCPointT::Ptr p_input,
                                                double p_radius,
                                                int p_minNN);

    void printToPCLViewer();

    void setPCLViewer();

    int loadFile(const std::string & p_string);

    void setShowUi(bool p_input);

    bool viewerStoped() const;

    void viewerSpinOnce() const;

    void spinOnceTestFile();


private:

    PCPointT::Ptr m_segmented_cloud;
    PCPointT::Ptr m_cloud;
    PCPointT::Ptr m_objects_cloud;
    pcl::PCLPointCloud2Ptr m_loaded_cloud;

    ros::Publisher m_pub;
    ros::Publisher m_pub2;
    ros::Publisher m_pub3;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_pclViewer;

    bool m_showUI;


};


#endif
