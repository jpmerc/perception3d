#ifndef fileAPI_H
#define fileAPI_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

struct Object{
    string name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud;
    geometry_msgs::PoseStampedConstPtr relative_arm_pose;
    geometry_msgs::PoseStampedConstPtr object_pose;
};


class fileAPI{
public:
    fileAPI(string directory);
    Object createObject(string name,pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,geometry_msgs::PoseStampedConstPtr relative_arm_pose,geometry_msgs::PoseStampedConstPtr object_pose);
    string findDefaultName(); //Parse the directory and find a new name for the object
    void saveObject(Object obj);
    void save(string name,pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,geometry_msgs::PoseStampedConstPtr relative_arm_pose,geometry_msgs::PoseStampedConstPtr object_pose);
    vector<Object> getAllObjects();
    Object getObjectByIndex(int index);

private:
    void parseDirectory(); //Set the index number (all files will be of the form index.txt -> check the highest index and assign to index variable)

    int highest_index;

};

#endif
