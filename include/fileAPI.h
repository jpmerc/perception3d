#ifndef FILEAPI_H
#define FILEAPI_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <jaco_custom.h>
#include <objectBd.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/cvfh.h>

#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>



class FileAPI{

public:
    FileAPI(const std::string& directory);

    ObjectBd createObject(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,
                          std::vector<tf::Transform> relative_arm_pose,
                          std::vector<tf::Transform> object_pose);

    ObjectBd createObject(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,
                          std::vector<tf::Transform> relative_arm_pose,
                          std::vector<tf::Transform> object_pose,
                          std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > p_tf);

    void save(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointCloud,
              std::vector<tf::Transform> relative_arm_pose,
              std::vector<tf::Transform> object_pose);

    void save(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointCloud,
              std::vector<tf::Transform> relative_arm_pose,
              std::vector<tf::Transform> object_pose,
              std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > p_tf);

    void saveObject(ObjectBd obj);

    ObjectBd loadFile(const std::string& p_fileName);


    std::vector<ObjectBd> getAllObjects() const;
    ObjectBd getObjectByIndex(int index) const;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr getAllHistograms() const;
    pcl::VFHSignature308 getHistogramByIndex (int p_index) const;

    ObjectBd retrieveObjectFromHistogram(int p_positionHisto) const;
    std::vector<ObjectBd> retrieveObjectFromHistogram(std::vector<int> indices) const;
    std::vector<int> retrieveHistogramFromObject(int p_indice) const;

    int fileAlreadyLoad(const std::string& p_filename);

private:

    std::string findDefaultName();

    void saveCvgh(ObjectBd p_obj, const std::string& p_fileName);
    void savePointCloud(ObjectBd p_obj, const std::string& p_fileName);
    void savePoseArm(ObjectBd p_obj, const std::string& p_fileName);
    void savePoseObject(ObjectBd p_obj, const std::string& p_fileName);
    void saveTranform(ObjectBd p_obj, const std::string& p_fileName);
    void failSaveUndo(const std::string& p_fileName);



    pcl::PointCloud<pcl::VFHSignature308>::Ptr loadSignature(const std::string& p_filename);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointCloud(const std::string& p_filename);
    std::vector<tf::Transform> loadPoseArm(const std::string& p_filename);
    std::vector<tf::Transform> loadPoseObject(const std::string& p_filename);
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > loadTransform(const std::string& p_filename);

    bool validationObj(ObjectBd p_obj);

    bool fileExist(const std::string& p_fileName);
    void parseDirectory();




    int m_highest_index;

    std::string m_pathToBd;
    std::string m_pathcvfh;
    std::string m_pathPointCloud;
    std::string m_pathPoseObject;
    std::string m_pathPoseArm;
    std::string m_pathTransform;

    std::vector<ObjectBd> m_bdObjectVector;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr m_pcvfh;

};

#endif
