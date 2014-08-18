#ifndef OBJECT_RECOGNITION_H
#define OBJECT_RECOGNITION_H


#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#include <fileAPI.h>


#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/impl/multiscale_feature_persistence.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <Eigen/Eigen>
#include <pcl/filters/voxel_grid.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <angles/angles.h>
#include <numeric>

typedef pcl::PointXYZRGB PointT;

class Object_recognition
{
public:
    Object_recognition();
//    Eigen::Matrix4f mergePointClouds(pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_src,
//                                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_target,
//                                     pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
//                                     pcl::PointCloud<PointT>::Ptr p_cloud_target_feature);

//    Eigen::Matrix4f mergePointCloudsShot(pcl::PointCloud<pcl::SHOT1344>::Ptr f_src,
//                                         pcl::PointCloud<pcl::SHOT1344>::Ptr f_target,
//                                        pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
//                                        pcl::PointCloud<PointT>::Ptr p_cloud_target_feature);

    Eigen::Matrix4f mergePointCVFH(pcl::PointCloud<pcl::VFHSignature308>::Ptr f_src,
                                   pcl::PointCloud<pcl::VFHSignature308>::Ptr f_target,
                                   pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
                                   pcl::PointCloud<PointT>::Ptr p_cloud_target_feature);

    double mergePointCVFH(pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
                          pcl::PointCloud<PointT>::Ptr p_cloud_target_feature,
                          Eigen::Matrix4f &transform_guess);

    double mergePointCVFH(pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
                          pcl::PointCloud<PointT>::Ptr p_cloud_target_feature,
                          Eigen::Matrix4f &transform_guess,
                          double &executionTime);

//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr calculateFPFH(pcl::PointCloud<PointT>::Ptr p_cloud,
//                                                             pcl::PointCloud<PointT>::Ptr p_feature,
//                                                             pcl::PointCloud<pcl::Normal>::Ptr p_normal);

//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr calculateFPFHUS(pcl::PointCloud<PointT>::Ptr p_cloud,
//                                                             pcl::PointCloud<pcl::Normal>::Ptr p_normal);

//    pcl::PointCloud<pcl::SHOT1344>::Ptr calculateShotColor(pcl::PointCloud<PointT>::Ptr p_cloud,
//                                                           pcl::PointCloud<PointT>::Ptr p_feature,
//                                                           pcl::PointCloud<pcl::Normal>::Ptr p_normal);

//    pcl::PointCloud<pcl::SHOT1344>::Ptr calculateShotColorUS(pcl::PointCloud<PointT>::Ptr p_cloud,
//                                                             pcl::PointCloud<pcl::Normal>::Ptr p_normal);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr calculateCVFH(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                               pcl::PointCloud<pcl::Normal>::Ptr p_normal,
                                                               std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr calculateCVFH(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                               pcl::PointCloud<pcl::Normal>::Ptr p_normal,
                                                               std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf,
                                                               std::vector<Eigen::Vector3f> &p_centroid);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr calculateCVFH(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                               pcl::PointCloud<pcl::Normal>::Ptr p_normal,
                                                               std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf,
                                                               std::vector<Eigen::Vector3f> &p_centroid,
                                                               std::vector<pcl::PointIndices>& p_indices);

    void compute_normal(pcl::PointCloud<PointT>::Ptr p_cloud, pcl::PointCloud<pcl::Normal>::Ptr p_normal);

    int object_recon(pcl::PointCloud<PointT>::Ptr p_cloud,
                      pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd);

    void showPointCloud(pcl::PointCloud<PointT>::Ptr p_cloud);

    void computeUniformSampling(pcl::PointCloud<PointT>::Ptr p_cloudIn,
                                pcl::PointCloud<PointT>::Ptr p_cloudOuput);

//    void fpfhProcessing(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
//                        pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr);

//    void shotColorProcessing(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
//                             pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr);

//    void usProcessingFpfh(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
//                          pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr);

//    void usProcessingShot(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
//                          pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr);

    void usProcessingCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
                          pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr);

    int histogramComparaison(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud,
                              pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud);

    int histogramComparison(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud,
                            pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud);

    std::vector<float> histogramComparisonVector(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud,
                                                 pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr makeCVFH(pcl::PointCloud<PointT>::Ptr p_cloud);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr makeCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
                                                        std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf_);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr makeCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
                                                        std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf_,
                                                        std::vector<Eigen::Vector3f>& p_centroid);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr makeCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
                                                        std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf_,
                                                        std::vector<Eigen::Vector3f>& p_centroid,
                                                        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& p_surface);

    void pointCloudExtractor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud,
                             pcl::PointIndices p_indices,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloudOut);

    int OURCVFHRecognition(pcl::PointCloud<PointT>::Ptr in_pc, FileAPI *fileAPI, Eigen::Matrix4f &trans);
    std::vector<double> OURCVFHRecognition(pcl::PointCloud<PointT>::Ptr in_pc, std::vector<pcl::PointCloud<PointT>::Ptr> hypotheses);


    std::vector<std::vector<int> > getNNSurfaces(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud,
                                                                     pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud,
                                                                     int NNnumber);

    pcl::PointCloud<PointT>::Ptr transformAndVoxelizePointCloud(pcl::PointCloud<PointT>::Ptr in_source, pcl::PointCloud<PointT>::Ptr in_target, Eigen::Matrix4f in_transform);

    double m_rmse_recognition_threshold;

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

    ros::Time us_time;
    ros::Time m_us_fpfh_time;
    ros::Time m_us_shot_time;

    ros::Time m_us_bd_time;
    ros::Time m_us_bd_fpfh_time;
    ros::Time m_us_bd_shot_time;

    //FileAPI m_bd;



};

#endif

