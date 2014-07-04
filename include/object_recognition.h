#ifndef object_recognition_H
#define object_recognition_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGB PointT;

class Object_recognition
{
public:
    Object_recognition();
    Eigen::Matrix4f mergePointClouds(pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_src,pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_target,
                                     pcl::PointCloud<PointT>::Ptr p_cloud_src_feature, pcl::PointCloud<PointT>::Ptr p_cloud_target_feature);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr calculateFPFH(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                             pcl::PointCloud<PointT>::Ptr p_feature,
                                                             pcl::PointCloud<pcl::Normal>::Ptr p_normal);

    void compute_normal(pcl::PointCloud<PointT>::Ptr p_cloud, pcl::PointCloud<pcl::Normal>::Ptr p_normal);

    bool object_recon(pcl::PointCloud<PointT>::Ptr p_cloud);


private:

    double m_sac_ia_maximum_distance;
    float m_sac_ia_minimum_sampling_distance;
    int m_sac_ia_maximum_iterations;
    int m_sac_ia_number_of_samples;
    int m_sac_ia_correspondance_randomness;
    float m_fpfh_persistence_scales[3];
    float m_fpfh_persistence_alpha;
    float m_normal_sphere_radius;

    double m_icp_fitness_score;

};

#endif
