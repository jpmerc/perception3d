#ifndef fileAPI_H
#define fileAPI_H

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

    void save(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointCloud,
              std::vector<tf::Transform> relative_arm_pose,
              std::vector<tf::Transform> object_pose);

    void saveObject(ObjectBd obj);

    ObjectBd loadFile(std::string p_fileName);


    std::vector<ObjectBd> getAllObjects() const;
    ObjectBd getObjectByIndex(int index) const;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr getAllHistograme() const;
    pcl::VFHSignature308 getHistogrameByIndex (int p_index) const;

    ObjectBd retrieveObjectFromHistogramme(int p_positionHisto);

    int fileAlreadyLoad(const std::string& p_filename);

private:

    std::string findDefaultName(); //Parse the directory and find a new name for the object

    void saveCvgh(ObjectBd p_obj, std::string p_fileName);
    void savePointCloud(ObjectBd p_obj, std::string p_fileName);
    void savePoseArm(ObjectBd p_obj, std::string p_fileName);
    void savePoseObject(ObjectBd p_obj, std::string p_fileName);
    void failSaveUndo(std::string p_fileName);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr loadSignature(std::string p_filename);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointCloud(std::string p_filename);
    std::vector<tf::Transform> loadPoseArm(std::string p_filename);
    std::vector<tf::Transform> loadPoseObject(std::string p_filename);

    bool fileExist(std::string p_fileName);
    void parseDirectory();




    int m_highest_index;

    std::string m_pathToBd;
    std::string m_pathcvfh;
    std::string m_pathPointCloud;
    std::string m_pathPoseObject;
    std::string m_pathPoseArm;

    std::vector<ObjectBd> m_bdObjectVector;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr m_pcvfh;

};

#endif
