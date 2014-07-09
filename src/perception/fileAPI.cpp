#include "fileAPI.h"

using namespace std;

FileAPI::FileAPI(const string & directory):
    m_pathToBd(directory),
    m_highest_index(1)
{
    m_pathcvfh = m_pathToBd + "/cvfh";
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
        pcl::PointCloud<pcl::VFHSignature308>::Ptr  signature_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
        while(it != boost::filesystem3::directory_iterator())
        {
            std::cout << *it << std::endl;
            path = *it;

            pcl::io::loadPCDFile(path.c_str(), *signature_ptr);
            objectName = path.stem().c_str();
            m_bdObjectVector.push_back(ObjectBd(objectName,
                                                signature_ptr));
            for(int i = 0; i < signature_ptr->size(); i++)
            {
                m_pcvfh->push_back(signature_ptr->at(i));
            }
            it++;
        }

        std::cout << "Point Cloud CBFH size : " << m_pcvfh->size() << std::endl;
        std::cout << "Object number loaded : " << m_bdObjectVector.size() << std::endl;
        std::cout << "Data base load finish" << std::endl;
    }
    else
    {
        std::cerr << "The path to the librairie doesn't exist" << std::endl;
        throw(runtime_error("load fail"));
    }

}

ObjectBd FileAPI::createObject(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,
                               std::vector<tf::Transform> relative_arm_pose,
                               std::vector<tf::Transform> object_pose)
{
    std::string objectName = findDefaultName();
    ObjectBd createdObject(objectName,
                           object_signature,
                           object_pointcloud,
                           relative_arm_pose,
                           object_pose);

    return createdObject;
}

string FileAPI::findDefaultName()
{
    parseDirectory();
    stringstream ss;
    ss << m_highest_index;
    std::cout << ss.str() << std::endl;

    return ss.str();

}

void FileAPI::saveObject(ObjectBd obj)
{
    if(obj.objectIsComplete())
    {
        std::string fileName = obj.getName();
        saveCvgh(obj, fileName);
        savePointCloud(obj, fileName);
        savePoseArm(obj, fileName);
        savePoseObject(obj, fileName);
        std::cout << "The file have been save under the name : " << fileName << std::endl;
    }
    else
    {
        ObjectBd loadedObj = loadFile(obj.getName());
        saveObject(loadedObj);
    }

}

void FileAPI::save(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointCloud,
                   std::vector<tf::Transform> relative_arm_pose,
                   std::vector<tf::Transform> object_pose)
{
    ObjectBd createdObject = createObject(object_signature,
                                          object_pointCloud,
                                          relative_arm_pose,
                                          object_pose);

    saveObject(createdObject);
}

void FileAPI::saveCvgh(ObjectBd p_obj, std::string p_fileName)
{
    boost::filesystem3::path path(m_pathcvfh);
    path /= p_fileName;
    path.replace_extension(".pcd");
    try
    {
        pcl::io::savePCDFile(path.c_str(), *(p_obj.getSignature()));
    }
    catch (pcl::IOException ex)
    {
        failSaveUndo(p_fileName);
    }
}

void FileAPI::savePointCloud(ObjectBd p_obj, std::string p_fileName)
{
    boost::filesystem3::path path(m_pathPointCloud);
    path /= p_fileName;
    path.replace_extension(".pcd");
    try
    {
        pcl::io::savePCDFile(path.c_str(), *(p_obj.getPointCloud()));
    }
    catch (pcl::IOException ex)
    {
        failSaveUndo(p_fileName);
    }
}

void FileAPI::savePoseArm(ObjectBd p_obj, std::string p_fileName)
{
    boost::filesystem3::path path(m_pathPoseArm);
    path /= p_fileName;
    path.replace_extension(".txt");
    //boost::filesystem3::ofstream ofs(path);
    //ofs.open(path, std::ios_base::app);
    std::ofstream ofs(path.c_str(), std::ios_base::app);
    if(ofs.is_open())
    {
        for(int i = 0; i < p_obj.getArmPose().size(); i++)
        {
            double roll, yaw, pitch;
            p_obj.getArmPose().at(i).getBasis().getEulerYPR(yaw, pitch, roll);
            ofs << p_obj.getArmPose().at(i).getOrigin().getX() << ";";
            ofs << p_obj.getArmPose().at(i).getRotation().getY() << ";";
            ofs << p_obj.getArmPose().at(i).getRotation().getZ() << ";";
            ofs << yaw << ";";
            ofs << pitch << ";";
            ofs << roll << std::endl;
        }
        ofs.close();
    }
    else
    {
        failSaveUndo(p_fileName);
    }
}

void FileAPI::savePoseObject(ObjectBd p_obj, std::string p_fileName)
{
    boost::filesystem3::path path(m_pathPoseObject);
    path /= p_fileName;
    path.replace_extension(".txt");
    //boost::filesystem3::ofstream ofs(path);
    //ofs.open(path, std::ios_base::app);

    std::ofstream ofs(path.c_str());
    if(ofs.is_open())
    {
        for(int i = 0; i < p_obj.getObjectPose().size(); i++)
        {
            double roll, yaw, pitch;
            p_obj.getObjectPose().at(i).getBasis().getEulerYPR(yaw, pitch, roll);
            ofs << p_obj.getObjectPose().at(i).getOrigin().getX() << ";";
            ofs << p_obj.getObjectPose().at(i).getRotation().getY() << ";";
            ofs << p_obj.getObjectPose().at(i).getRotation().getZ() << ";";
            ofs << yaw << ";";
            ofs << pitch << ";";
            ofs << roll << std::endl;
        }
        ofs.close();
    }
    else
    {
        failSaveUndo(p_fileName);
    }
}

void FileAPI::failSaveUndo(string p_fileName)
{
    boost::filesystem3::path path(m_pathToBd);
    boost::filesystem3::recursive_directory_iterator dirIt(path);
    boost::filesystem3::path tempPath;

    while(dirIt != boost::filesystem3::recursive_directory_iterator())
    {
        tempPath = *dirIt;
        if(tempPath.stem() == p_fileName and boost::filesystem3::exists(tempPath))
        {
            boost::filesystem3::remove(tempPath);
            std::cout << tempPath.c_str() << "  have been delete" << std::endl;
        }

        dirIt++;
    }
}


ObjectBd FileAPI::retrieveObjectFromHistogramme(int p_positionHisto)
{
    int sommeCVFH = 0;

    for(int i = 0; i < m_bdObjectVector.size(); i++)
    {
        sommeCVFH += m_bdObjectVector.at(i).getSize();

        if(sommeCVFH >= p_positionHisto)
        {
            return m_bdObjectVector.at(i);
        }
    }
}

vector<ObjectBd> FileAPI::getAllObjects() const
{
    return m_bdObjectVector;
}

ObjectBd FileAPI::getObjectByIndex(int index) const
{
    return m_bdObjectVector.at(index);
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr FileAPI::getAllHistograme() const
{
    return m_pcvfh;
}

pcl::VFHSignature308 FileAPI::getHistogrameByIndex(int p_index) const
{
    return m_pcvfh->at(p_index);
}

void FileAPI::parseDirectory()
{
    boost::filesystem3::path path(m_pathcvfh);
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
        dirIt++;
    }
    m_highest_index++;
}

bool FileAPI::fileExist(string p_fileName)
{

    bool fileCvfh = false;
    bool filePointCloud = false;
    bool filePoseArm = false;
    bool filePoseObject = false;



    boost::filesystem3::path path(m_pathcvfh);

    path /= p_fileName;
    path.replace_extension(".pcd");
    fileCvfh = boost::filesystem3::exists(path);

    path = m_pathPointCloud;
    path /= p_fileName;
    path.replace_extension(".pcd");
    filePointCloud = boost::filesystem3::exists(path);

    path = m_pathPoseObject;
    path /= p_fileName;
    path.replace_extension(".txt");
    filePoseObject = boost::filesystem3::exists(path);

    path = m_pathPoseArm;
    path /= p_fileName;
    path.replace_extension(".txt");
    filePoseArm = boost::filesystem3::exists(path);


    return fileCvfh and filePointCloud and filePoseArm and filePoseObject;
}

ObjectBd FileAPI::loadFile(string p_fileName)
{
    if(fileExist(p_fileName))
    {

        pcl::PointCloud<pcl::VFHSignature308>::Ptr signature_ptr =loadSignature(p_fileName);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = loadPointCloud(p_fileName);
        std::vector<tf::Transform> arm = loadPoseArm(p_fileName);
        std::vector<tf::Transform> object = loadPoseObject(p_fileName);
        ObjectBd returnedObject(p_fileName,
                                signature_ptr,
                                cloud_ptr,
                                arm,
                                object);
        return returnedObject;
    }
    else
    {
        //gestion exception....
    }
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr FileAPI::loadSignature(string p_filename)
{
    pcl::PointCloud<pcl::VFHSignature308>::Ptr signature_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
    boost::filesystem3::path path(m_pathcvfh);
    path /= p_filename;
    path.replace_extension(".pcd");
    pcl::io::loadPCDFile(path.c_str(), *signature_ptr);
    return signature_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FileAPI::loadPointCloud(string p_filename)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::filesystem3::path path(m_pathPointCloud);
    path /= p_filename;
    path.replace_extension(".pcd");
    pcl::io::loadPCDFile(path.c_str(), *cloud_ptr);
    return cloud_ptr;
}

std::vector<tf::Transform> FileAPI::loadPoseArm(string p_filename)
{
    boost::filesystem3::path path(m_pathPoseArm);
    path /= p_filename;
    path.replace_extension(".txt");
    boost::filesystem3::ifstream ifs(path);
    ifs.open(path, std::ios_base::in);
    if(ifs.is_open())
    {
        std::vector<tf::Transform> returnVector;
        std::string line;
        while(getline(ifs,line))
        {
            std::vector<string> splitterVector;
            boost::algorithm::split(splitterVector, line, boost::algorithm::is_any_of(";"));
            std::vector<double> doubleVetor;
            for(int i = 0; i < splitterVector.size(); i++)
            {
                doubleVetor[i] = atof(splitterVector[i].c_str());
            }
            tf::Transform tf;
            tf::Vector3 vec(doubleVetor[0], doubleVetor[1], doubleVetor[2]);
            tf.setOrigin(vec);
            tf::Quaternion quat;
            quat.setRPY(doubleVetor[5], doubleVetor[4], doubleVetor[3]);
            tf.setRotation(quat);
            returnVector.push_back(tf);
        }
        return returnVector;
    }
}

std::vector<tf::Transform> FileAPI::loadPoseObject(string p_filename)
{
    boost::filesystem3::path path(m_pathPoseObject);
    path /= p_filename;
    path.replace_extension(".txt");
    boost::filesystem3::ifstream ifs(path);
    ifs.open(path, std::ios_base::in);
    if(ifs.is_open())
    {
        std::vector<tf::Transform> returnVector;
        std::string line;
        while(getline(ifs,line))
        {
            std::vector<string> splitterVector;
            boost::algorithm::split(splitterVector, line, boost::algorithm::is_any_of(";"));
            std::vector<double> doubleVetor;
            for(int i = 0; i < splitterVector.size(); i++)
            {
                doubleVetor[i] = atof(splitterVector[i].c_str());
            }
            tf::Transform tf;
            tf::Vector3 vec(doubleVetor[0], doubleVetor[1], doubleVetor[2]);
            tf.setOrigin(vec);
            tf::Quaternion quat;
            quat.setRPY(doubleVetor[5], doubleVetor[4], doubleVetor[3]);
            tf.setRotation(quat);
            returnVector.push_back(tf);
        }
        return returnVector;
    }
}
