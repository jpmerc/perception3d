#include "fileAPI.h"

using namespace std;

FileAPI::FileAPI(const string & directory):
    m_pathToBd(directory)
{
    m_pathcvfh = m_pathToBd + "/signature";
    m_pathPointCloud = m_pathToBd + "/pointCloud";
    m_pathPoseObject = m_pathToBd +"/poseObject";
    m_pathPoseArm = m_pathToBd +"/poseArm";
    m_pcvfh.reset(new pcl::PointCloud<pcl::VFHSignature308>);
    boost::filesystem3::path directory_path(m_pathcvfh);
    boost::filesystem3::directory_iterator it(directory_path);
    boost::filesystem3::path path;
    if (boost::filesystem3::exists(directory_path))
    {
        std::string objectName;
        pcl::PointCloud<pcl::VFHSignature308>::Ptr  signature(new pcl::PointCloud<pcl::VFHSignature308>);
        while(it != boost::filesystem3::directory_iterator())
        {
            std::cout << *it << std::endl;
            path = *it;

            pcl::io::loadPCDFile(path.c_str(), *signature);
            objectName = path.filename().c_str();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcObject_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            geometry_msgs::PoseStampedConstPtr temp_ptr1(new geometry_msgs::PoseStamped);
            geometry_msgs::PoseStampedConstPtr temp_ptr2(new geometry_msgs::PoseStamped);
            m_bdObjectVector.push_back(createObject(objectName,
                                                    signature,
                                                    pcObject_ptr,
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
    else
    {
        std::cerr << "The path to the librairie doesn't exist" << std::endl;
        throw(runtime_error("load fail"));
    }

}

Object FileAPI::createObject(string name,
                             pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,
                             geometry_msgs::PoseStampedConstPtr relative_arm_pose,
                             geometry_msgs::PoseStampedConstPtr object_pose)
{
    Object createdObject;
    createdObject.name = name;
    createdObject.pcSize = object_signature->size();
    createdObject.object_signature = object_signature;
    createdObject.object_point_cloud = object_pointcloud;
    createdObject.object_pose = object_pose;
    createdObject.relative_arm_pose = relative_arm_pose;

    return createdObject;
}

string FileAPI::findDefaultName()
{
    parseDirectory();
    stringstream ss;
    ss << m_highest_index;

    return ss.str();

}

void FileAPI::saveObject(Object obj)
{
    boost::filesystem3::path path(m_pathToBd);
    std::string fileName = findDefaultName();
    path/= fileName;
    path.replace_extension(".pcd");
    pcl::io::savePCDFileASCII(path.c_str(), *(obj.object_signature));
}

void FileAPI::save(string name,
                   pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointCloud,
                   geometry_msgs::PoseStampedConstPtr relative_arm_pose,
                   geometry_msgs::PoseStampedConstPtr object_pose)
{
    Object obj;
    obj.name = name;
    obj.pcSize = object_signature->size();
    obj.object_signature = object_signature;
    obj.object_point_cloud = object_pointCloud;
    obj.relative_arm_pose = relative_arm_pose;
    obj.object_pose = object_pose;
    saveObject(obj);
}

Object FileAPI::retrieveObjectFromHistogramme(int p_positionHisto)
{
    int sommeCVFH = 0;

    for(int i = 0; i < m_bdObjectVector.size(); i++)
    {
        sommeCVFH += m_bdObjectVector.at(i).pcSize;

        if(sommeCVFH >= p_positionHisto)
        {
            return m_bdObjectVector.at(i);
        }
    }
}

vector<Object> FileAPI::getAllObjects()
{
    return m_bdObjectVector;
}

Object FileAPI::getObjectByIndex(int index)
{
    return m_bdObjectVector.at(index);
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr FileAPI::getAllHistograme()
{
    return m_pcvfh;
}

pcl::VFHSignature308 FileAPI::getHistogrameByIndex(int p_index)
{
    return m_pcvfh->at(p_index);
}

void FileAPI::parseDirectory()
{
    boost::filesystem3::path path(m_pathToBd);
    boost::filesystem3::directory_iterator dirIt(path);
    boost::filesystem3::path pathTemp;

    while(dirIt != boost::filesystem3::directory_iterator())
    {
        pathTemp = *dirIt;

        int currentfile = atoi(pathTemp.stem().c_str());
        if(currentfile > m_highest_index)
        {
            m_highest_index = currentfile;
        }
    }
}
