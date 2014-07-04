#include "fileAPI.h"


fileAPI::fileAPI(string directory){

}

Object fileAPI::createObject(string name,pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,geometry_msgs::PoseStampedConstPtr relative_arm_pose,geometry_msgs::PoseStampedConstPtr object_pose){

}

string fileAPI::findDefaultName(){

}

void fileAPI::saveObject(Object obj){

}

void fileAPI::save(string name,pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,geometry_msgs::PoseStampedConstPtr relative_arm_pose,geometry_msgs::PoseStampedConstPtr object_pose){
    Object obj;
    obj.name = name;
    obj.object_pointcloud = object_pointcloud;
    obj.relative_arm_pose = relative_arm_pose;
    obj.object_pose = object_pose;
    saveObject(obj);
}

vector<Object> fileAPI::getAllObjects(){

}

Object fileAPI::getObjectByIndex(int index){

}

void fileAPI::parseDirectory(){

}
