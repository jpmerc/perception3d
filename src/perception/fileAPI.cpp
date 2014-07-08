#include "fileAPI.h"

using namespace std;

FileAPI::FileAPI(const string & directory):
    m_pathToBd(directory)
{
    m_pcvfh.reset(new pcl::PointCloud<pcl::VFHSignature308>);
    boost::filesystem3::path directory_path(directory);
    boost::filesystem3::directory_iterator it(directory_path);
    boost::filesystem3::path path;

    std::string objectName;
    int objectSize;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr  signature(new pcl::PointCloud<pcl::VFHSignature308>);
    while(it != boost::filesystem3::directory_iterator())
    {
        std::cout << *it << std::endl;
        path = *it;

        pcl::io::loadPCDFile(path.c_str(), *signature);
        objectName = path.filename().c_str();
        objectSize = signature->size();
        geometry_msgs::PoseStampedConstPtr temp_ptr1(new geometry_msgs::PoseStamped);
        geometry_msgs::PoseStampedConstPtr temp_ptr2(new geometry_msgs::PoseStamped);
        m_bdObjectVector.push_back(createObject(objectName,
                                                objectSize,
                                                signature,
                                                temp_ptr1,
                                                temp_ptr2));
        for(int i = 0; i < signature->size(); i++)
        {
            m_pcvfh->push_back(signature->at(i));
        }
        it++;
    }

    std::cout << "Point Cloud CBFH size : " << m_pcvfh->size() << std::endl;
    std::cout << "Object number loaded : " << m_bdObjectVector.size() << std::endl;
    std::cout << "Data base load finis" << std::endl;

}

Object FileAPI::createObject(string name,
                             int size,
                             pcl::PointCloud<pcl::VFHSignature308>::Ptr object_pointcloud,
                             geometry_msgs::PoseStampedConstPtr relative_arm_pose,
                             geometry_msgs::PoseStampedConstPtr object_pose)
{
    Object createdObject;
    createdObject.name = name;
    createdObject.pcSize = size;
    createdObject.object_signature = object_pointcloud;
    createdObject.object_pose = object_pose;
    createdObject.relative_arm_pose = relative_arm_pose;

    return createdObject;
}

string FileAPI::findDefaultName(){

}

void FileAPI::saveObject(Object obj){

}

void FileAPI::save(string name,pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,geometry_msgs::PoseStampedConstPtr relative_arm_pose,geometry_msgs::PoseStampedConstPtr object_pose){
    Object obj;
    obj.name = name;
    *(obj.object_signature) = *object_signature;
    obj.relative_arm_pose = relative_arm_pose;
    obj.object_pose = object_pose;
    saveObject(obj);
}

vector<Object> FileAPI::getAllObjects(){

}

Object FileAPI::getObjectByIndex(int index){

}

pcl::PointCloud<pcl::VFHSignature308>::Ptr FileAPI::getAllHistograme()
{
    return m_pcvfh;
}

pcl::VFHSignature308 FileAPI::getHistogrameByIndex(int p_index)
{
    return m_pcvfh->at(p_index);
}

void FileAPI::parseDirectory(){

}
