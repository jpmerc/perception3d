#ifndef OBJECTBD_H
#define OBJECTBD_H

#include <pcl/point_cloud.h>
#include <pcl/features/cvfh.h>
#include <pcl/point_types.h>
#include <jaco_custom.h>

class ObjectBd
{

public:

    ObjectBd(std::string p_name,
             pcl::PointCloud<pcl::VFHSignature308>::Ptr p_signature,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pointCloud,
             std::vector<tf::Transform> p_armPose,
             std::vector<tf::Transform> p_objectPose);
    ObjectBd(std::string p_name,
             pcl::PointCloud<pcl::VFHSignature308>::Ptr p_signature);

    std::string getName() const;
    int getSize() const;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr getSignature() const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud() const;
    std::vector<tf::Transform> getArmPose() const;
    std::vector<tf::Transform> getObjectPose() const;
    bool objectIsComplete() const;

    bool setAllAttribut(std::string p_name,
                        pcl::PointCloud<pcl::VFHSignature308>::Ptr p_signature,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pointCloud,
                        std::vector<tf::Transform> p_armPose,
                        std::vector<tf::Transform> p_objectPose);

    ObjectBd& operator = (const ObjectBd& p_object);
    bool operator == (const ObjectBd& p_object);



private:

    bool m_fullLoaded;
    std::string m_name;
    int m_pcSize;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr m_object_signature;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_object_point_cloud;
    std::vector<tf::Transform> m_relative_arm_pose;
    std::vector<tf::Transform> m_object_pose;

};

#endif
