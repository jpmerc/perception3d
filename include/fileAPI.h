#ifndef fileAPI_H
#define fileAPI_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/cvfh.h>


struct Object{
    std::string name;
    int pcSize;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature;
    geometry_msgs::PoseStampedConstPtr relative_arm_pose;
    geometry_msgs::PoseStampedConstPtr object_pose;
};


class FileAPI{
public:
    FileAPI(const std::string& directory = "/home/robot/histoGrammeBD/BD");
    Object createObject(std::string name,
                        int size,
                        pcl::PointCloud<pcl::VFHSignature308>::Ptr object_pointcloud,
                        geometry_msgs::PoseStampedConstPtr relative_arm_pose,
                        geometry_msgs::PoseStampedConstPtr object_pose);
    std::string findDefaultName(); //Parse the directory and find a new name for the object
    void saveObject(Object obj);
    void save(std::string name,pcl::PointCloud<pcl::VFHSignature308>::Ptr object_pointcloud,geometry_msgs::PoseStampedConstPtr relative_arm_pose,geometry_msgs::PoseStampedConstPtr object_pose);
    std::vector<Object> getAllObjects();
    Object getObjectByIndex(int index);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr getAllHistograme();
    pcl::VFHSignature308 getHistogrameByIndex (int p_index);

    Object retrieveObjectFromHistogramme(int p_positionHisto);

private:
    void parseDirectory(); //Set the index number (all files will be of the form index.txt -> check the highest index and assign to index variable)

    int highest_index;

    std::string m_pathToBd;
    std::vector<Object> m_bdObjectVector;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr m_pcvfh;

};

#endif
