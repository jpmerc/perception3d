#include "fileAPI.h"

using namespace std;

FileAPI::FileAPI(const string & directory)
{
    boost::filesystem3::path directory_path(directory);
    boost::filesystem3::directory_iterator it(directory_path);

}

Object FileAPI::createObject(string name,pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,geometry_msgs::PoseStampedConstPtr relative_arm_pose,geometry_msgs::PoseStampedConstPtr object_pose){

}

string FileAPI::findDefaultName(){

}

void FileAPI::saveObject(Object obj){

}

void FileAPI::save(string name,pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,geometry_msgs::PoseStampedConstPtr relative_arm_pose,geometry_msgs::PoseStampedConstPtr object_pose){
    Object obj;
    obj.name = name;
    obj.object_pointcloud = object_pointcloud;
    obj.relative_arm_pose = relative_arm_pose;
    obj.object_pose = object_pose;
    saveObject(obj);
}

vector<Object> FileAPI::getAllObjects(){

}

Object FileAPI::getObjectByIndex(int index){

}

void FileAPI::parseDirectory(){

}
