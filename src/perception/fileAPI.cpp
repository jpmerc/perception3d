#include "fileAPI.h"

using namespace std;

/*
  The default constructor.
  param[in] directory take a string to the root directory of the BD.  The bd must be organise like
  this root/cvfh, root/pointCloud, root/poseObject, root/poseArm.  Will load all the object and the
  pose.
  */

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
    //boost::filesystem3::directory_iterator it(directory_path);
    //boost::filesystem3::path path;


    if (boost::filesystem3::exists(directory_path))
    {
        boost::filesystem3::directory_iterator it(directory_path);
        boost::filesystem3::path path;
        pcl::PointCloud<pcl::VFHSignature308>::Ptr  signature_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
        while(it != boost::filesystem3::directory_iterator())
        {
            std::cout << *it << std::endl;
            path = *it;

            ObjectBd objTemp = loadFile(path.stem().c_str());
            m_bdObjectVector.push_back(objTemp);

            pcl::io::loadPCDFile(path.c_str(), *signature_ptr);
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
       // throw(runtime_error("load fail"));
    }

}

/*
  The function to create a new object.  Will make sure that the new name is consitant for the BD.
  It is the only fonction to create an object and ensure no name clash.
  param[in] object_signature the OURCVFH signature for the object.
  param[in] object_pointCloud the original point cloud for the object.
  param[in] relative_arm_pose the the arm pose for the object
  param[in] object_pose the pose for the object
  */


ObjectBd FileAPI::createObject(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,
                               std::vector<tf::Transform> relative_arm_pose,
                               std::vector<tf::Transform> object_pose)
{
    if(object_signature->size() <= 0)
    {
        throw (std::runtime_error("The object don't have a signature"));
    }
    else if(object_pointcloud->size() <= 0)
    {
        throw (std::runtime_error("The point cloud is empty"));
    }
    else if(relative_arm_pose.size() <= 0)
    {
        throw(std::runtime_error("The object have no arm pose"));
    }
    else if(object_pose.size() <= 0)
    {
        throw(std::runtime_error("The object have no pose"));
    }
    else
    {
        std::string objectName = findDefaultName();
        ObjectBd createdObject(objectName,
                               object_signature,
                               object_pointcloud,
                               relative_arm_pose,
                               object_pose);

        return createdObject;
    }
}

/*
  The funtion will find the next name unused
  */

string FileAPI::findDefaultName()
{
    parseDirectory();
    stringstream ss;
    ss << m_highest_index;
    std::cout << ss.str() << std::endl;

    return ss.str();

}

/*
  The function to save an object.
  param[in] obj the object to save on BD.
  If the object is incomplete, the API will try to load it.  If it exist the process will go on.
  If the object is new an error will rise.
  */

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
        std::cout << "the object is incomplete" << std::endl;
        ObjectBd loadedObj = loadFile(obj.getName());
        saveObject(loadedObj);
    }

}

/*
  The function to save a new object.  Will creat a new object with a good name and will save it.
  */

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

/*
  A private function that save the signature at the good place in the BD.
  */

void FileAPI::saveCvgh(ObjectBd p_obj, const std::string& p_fileName)
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

/*
  A private function that save the point cloud in the BD.
  */

void FileAPI::savePointCloud(ObjectBd p_obj, const std::string& p_fileName)
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

/*
  The private function to save the pose arm in the BD.
  */

void FileAPI::savePoseArm(ObjectBd p_obj, const std::string& p_fileName)
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

/*
  The private function to save the object in the BD.
  */

void FileAPI::savePoseObject(ObjectBd p_obj, const std::string& p_fileName)
{
    boost::filesystem3::path path(m_pathPoseObject);
    path /= p_fileName;
    path.replace_extension(".txt");
    //boost::filesystem3::ofstream ofs(path);
    //ofs.open(path, std::ios_base::app);

    std::ofstream ofs(path.c_str(), std::ios::app);
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

/*
  The function that erase a partial save to keep the BD coherent.  Will parse all the BD and erase
  the file with the file name.
  */

void FileAPI::failSaveUndo(const std::string& p_fileName)
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

/*
  Function to retrieve the object from the histogram vector.
  param[in] p_positionHisto an index from the signature vector.
  return the object to which histogram correspond.  An object can have more than one histogram,
  the function compute the difference.
  */

ObjectBd FileAPI::retrieveObjectFromHistogram(int p_positionHisto)
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

/*
  return a vector of ObjectBd at the indices pass in parameter.
  */

vector<ObjectBd> FileAPI::retrieveObjectFromHistogram(vector<int> indices){
    vector<ObjectBd> object_vector;
    for(int i=0; i<indices.size(); i++){
        ObjectBd obj = retrieveObjectFromHistogram(indices.at(i));
        object_vector.push_back(obj);
    }
    return object_vector;
}

/*
  return the object vector.  The function is const.
  */


vector<ObjectBd> FileAPI::getAllObjects() const
{
    return m_bdObjectVector;
}

/*
  Return the object at the indice.  The function is const.
  */

ObjectBd FileAPI::getObjectByIndex(int index) const
{
    if(index < m_bdObjectVector.size() and index >= 0)
    {
        return m_bdObjectVector.at(index);
    }
    else
        throw(std::out_of_range("The index is out of range"));
}

/*
  Return the pointcloud containing the signature.  The function is const.
  */

pcl::PointCloud<pcl::VFHSignature308>::Ptr FileAPI::getAllHistograms() const
{
    return m_pcvfh;
}

/*
  Return the histogram at the indice.  The function is const.
  */

pcl::VFHSignature308 FileAPI::getHistogramByIndex(int p_index) const
{
    if(p_index < m_pcvfh->size() and p_index >= 0)
        return m_pcvfh->at(p_index);
    else
    {
        throw(std::out_of_range("The index is out of range"));
    }
}

/*
  A function use in to update the attribut m_highest_index.  Which is the higher name currently use.
  */

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

/*
  Function to look if the object exist in the BD, will look for the 4 files thats is needed.
  param[in] p_fileName the the filename to seach for.
  */

bool FileAPI::fileExist(const std::string& p_fileName)
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

/*
  Function to load a file.  Will look if the file exist and if the file is already loaded.
  param[in] p_filename the file to load.
  return an ObjectBd
  */

ObjectBd FileAPI::loadFile(const std::string& p_fileName)
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

            int fileLoaded = fileAlreadyLoad(p_fileName);
            if(fileLoaded == -1)
            {
                for(int i = 0; i < signature_ptr->size(); i++)
                {
                    m_pcvfh->push_back(signature_ptr->at(i));
                }
                m_bdObjectVector.push_back(returnedObject);
            }

            return returnedObject;
        }
    else
    {
        throw(std::runtime_error("The file don't exist"));
    }
}

/*
  Load a pcd file that contain the OURCVFH signature of an object.
  param[in] the file name.
  return a point cloud with the signature in it.
  */

pcl::PointCloud<pcl::VFHSignature308>::Ptr FileAPI::loadSignature(const std::string& p_filename)
{
    pcl::PointCloud<pcl::VFHSignature308>::Ptr signature_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
    boost::filesystem3::path path(m_pathcvfh);
    path /= p_filename;
    path.replace_extension(".pcd");
    pcl::io::loadPCDFile(path.c_str(), *signature_ptr);
    return signature_ptr;
}

/*
  Load a pcd file that contain the point cloud of an object.
  param[in] the file name.
  return a point cloud.
  */

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FileAPI::loadPointCloud(const std::string& p_filename)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::filesystem3::path path(m_pathPointCloud);
    path /= p_filename;
    path.replace_extension(".pcd");
    pcl::io::loadPCDFile(path.c_str(), *cloud_ptr);
    return cloud_ptr;
}

/*
  Load a txt file that contain the the poseArm.
  param[in] the file name.
  Return a vector with the transform in it.
  */

std::vector<tf::Transform> FileAPI::loadPoseArm(const std::string& p_filename)
{
    boost::filesystem3::path path(m_pathPoseArm);
    path /= p_filename;
    path.replace_extension(".txt");
    //boost::filesystem3::ifstream ifs(path);
    //ifs.open(path, std::ios_base::in);

    std::ifstream ifs(path.c_str());
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
                doubleVetor.push_back(atof(splitterVector[i].c_str()));
            }
            tf::Transform tf;
            tf::Vector3 vec(doubleVetor[0], doubleVetor[1], doubleVetor[2]);
            tf.setOrigin(vec);
            tf::Quaternion quat;
            quat.setRPY(doubleVetor[5], doubleVetor[4], doubleVetor[3]);
            tf.setRotation(quat);
            returnVector.push_back(tf);
        }
        ifs.close();
        return returnVector;
    }
}

/*
  Load a txt file that contain the the poseObject.
  param[in] the file name.
  Return a vector with the transform in it.
  */

std::vector<tf::Transform> FileAPI::loadPoseObject(const std::string& p_filename)
{
    boost::filesystem3::path path(m_pathPoseObject);
    path /= p_filename;
    path.replace_extension(".txt");
    //boost::filesystem3::ifstream ifs(path);
    //ifs.open(path, std::ios_base::in);
    std::ifstream ifs(path.c_str());
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
                doubleVetor.push_back(atof(splitterVector[i].c_str()));
            }
            tf::Transform tf;
            tf::Vector3 vec(doubleVetor[0], doubleVetor[1], doubleVetor[2]);
            tf.setOrigin(vec);
            tf::Quaternion quat;
            quat.setRPY(doubleVetor[5], doubleVetor[4], doubleVetor[3]);
            tf.setRotation(quat);
            returnVector.push_back(tf);
        }
        ifs.close();
        return returnVector;
    }
}

/*
  Parse the object vector to find if the object is already load.
  pram[in] p_filename the name of the object.
  return the index if the file is already loaded.  -1 if the file is not loaded.
  */


int FileAPI::fileAlreadyLoad(const string &p_filename)
{
    int position = -1;
    for(int i = 0; i < m_bdObjectVector.size(); i++)
    {
        if(m_bdObjectVector[i].getName() == p_filename)
            position = i;
    }
    return position;
}
