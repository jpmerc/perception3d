#include "fileAPI.h"

using namespace std;

/*
  Presently, there is two versions of the API.  One version have the tf and the other dont.  It was
  for backward compatibility during test.  Eventualy the version with no tf will not be used.
  Currently I don't use it anywere in my code and ObjectBd is fully compatible with the two versions.
  */




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
    m_pathTransform = m_pathToBd + "/transform";
    m_pcvfh.reset(new pcl::PointCloud<pcl::VFHSignature308>);

    boost::filesystem3::path directory_path(m_pathcvfh);

    if (boost::filesystem3::exists(directory_path))
    {
        boost::filesystem3::directory_iterator it(directory_path);
        boost::filesystem3::path path;
        while(it != boost::filesystem3::directory_iterator())
        {
            std::cout << *it << std::endl;
            path = *it;

            loadFile(path.stem().c_str());

            it++;
        }

        std::cout << "Point Cloud CVFH size : " << m_pcvfh->size() << std::endl;
        std::cout << "Object number loaded : " << m_bdObjectVector.size() << std::endl;
        std::cout << "Data base load finish" << std::endl;
    }
    else
    {
        std::cerr << "The path to the librairie doesn't exist" << std::endl;
        //throw(runtime_error("load fail"));
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
  The function to create a new object.  Will make sure that the new name is consitant for the BD.
  It is the only fonction to create an object and ensure no name clash.
  param[in] object_signature the OURCVFH signature for the object.
  param[in] object_pointCloud the original point cloud for the object.
  param[in] relative_arm_pose the the arm pose for the object
  param[in] object_pose the pose for the object
  param[in] p_tf the vector containing the tf from the OURCVFH
  */

ObjectBd FileAPI::createObject(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointcloud,
                               std::vector<tf::Transform> relative_arm_pose,
                               std::vector<tf::Transform> object_pose,
                               std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > p_tf)
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
                               object_pose,
                               p_tf);

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
  Look at all the attributs to see if they are all valid.
  The point cloud and the signature size must be > 0.
  The name should not be empty.
  The poses should not be empty.
  The transform should not be empty.
  */

bool FileAPI::validationObj(ObjectBd p_obj)
{
    bool validation = true;

    if(p_obj.getSignature()->size() <= 0)
    {
        validation = false;
        std::cout << "The signature must not be empty";
    }
    else if(p_obj.getPointCloud()->size() <= 0)
    {
        validation = false;
        std::cerr << "The point cloud must not be empty";
    }
    else if(p_obj.getName().empty())
    {
        validation = false;
        std::cerr << "The object must have a name";
    }
    else if(p_obj.getObjectPose().empty())
    {
        validation = false;
        std::cerr << "The object must have an object pose";
    }
    else if(p_obj.getArmPose().empty())
    {
        validation = false;
        std::cerr << "The object must have an arm pose";
    }
    else if(p_obj.getTransforms().empty())
    {
        validation = false;
        std::cerr << "The object must have a transform";
    }

    return validation;
}


/*
  The function to save an object.
  param[in] obj the object to save on BD.
  If the object is incomplete, the API will try to load it.  If it exist the process will go on.
  If the object is new an error will rise.
  */

void FileAPI::saveObject(ObjectBd p_obj)
{
    if(validationObj(p_obj))
    {
        std::string fileName = p_obj.getName();
        saveCvgh(p_obj, fileName);
        savePointCloud(p_obj, fileName);
        savePoseArm(p_obj, fileName);
        savePoseObject(p_obj, fileName);
        saveTranform(p_obj, fileName);

        int indice = fileAlreadyLoad(fileName);
        if(indice > -1 and indice < m_bdObjectVector.size())
        {
            std::vector<int> indices = retrieveHistogramFromObject(indice);
            m_pcvfh->erase(m_pcvfh->begin() + indices[0], m_pcvfh->begin() + indices[1]+1);
            m_bdObjectVector.erase(m_bdObjectVector.begin()+indice);
            m_bdObjectVector.push_back(p_obj);
            for(int i = 0; i < p_obj.getSignature()->size(); i++)
            {
                m_pcvfh->push_back(p_obj.getSignature()->at(i));
            }
            std::cout << "The file have been save under the name : " << fileName << std::endl;
        }
        else if(indice == -1)
        {
            m_bdObjectVector.push_back(p_obj);
            for(int i = 0; i < p_obj.getSignature()->size(); i++)
            {
                m_pcvfh->push_back(p_obj.getSignature()->at(i));
            }
            std::cout << "The file have been save under the name : " << fileName << std::endl;
        }
        else
        {
            failSaveUndo(p_obj.getName());
            std::cerr << "The index must be in range of the ObjectVector" << std::endl;
            throw(std::out_of_range("Trying to acces out of range on m_bdObjectVector"));
        }
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
  The function to save a new object.  Will creat a new object with a good name and will save it.
  */

void FileAPI::save(pcl::PointCloud<pcl::VFHSignature308>::Ptr object_signature,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pointCloud,
                   std::vector<tf::Transform> relative_arm_pose,
                   std::vector<tf::Transform> object_pose,
                   std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > p_tf)
{
    ObjectBd createdObject = createObject(object_signature,
                                          object_pointCloud,
                                          relative_arm_pose,
                                          object_pose,
                                          p_tf);

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
    std::ofstream ofs(path.c_str(), std::ios_base::out);
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
    std::ofstream ofs(path.c_str(), std::ios::out);
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

void FileAPI::saveTranform(ObjectBd p_obj, const string &p_fileName)
{
    boost::filesystem3::path path(m_pathTransform);
    path /= p_fileName;
    path.replace_extension(".txt");

    std::ofstream ofs(path.c_str(), std::ios::app);
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > tfV = p_obj.getTransforms();
    if(ofs.is_open())
    {
        for(int i = 0; i < tfV.size(); i ++)
        {
            Eigen::Matrix4f matrix = tfV.at(i);
            float* array;
            array = matrix.data();
            for(int i = 0; i < 16; i++)
            {
                if(i == 15)
                    ofs << *(array + i);
                else
                    ofs << *(array + i) << ";";
            }
            ofs << std::endl;
        }
    }
    ofs.close();
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

ObjectBd FileAPI::retrieveObjectFromHistogram(int p_positionHisto) const
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

vector<ObjectBd> FileAPI::retrieveObjectFromHistogram(vector<int> indices) const
{
    vector<ObjectBd> object_vector;
    for(int i=0; i<indices.size(); i++){
        ObjectBd obj = retrieveObjectFromHistogram(indices.at(i));
        object_vector.push_back(obj);
    }
    return object_vector;
}

/*
  Find the indice for an object signature.
  param[in] p_indice the object indice in his vector the max is size()-1.
  return a vector.  The 1rst value is the first indice.  The second value is the last indice.
  The value returned is an indice to be used in the m_pcvfh.  The value is between 0 and
  m_pcvfh.size().
  */

std::vector<int> FileAPI::retrieveHistogramFromObject(int p_indice) const
{
    int sommeSignature = 0;
    std::vector<int> indiceCompteur;
    for(int i = 0; i <= p_indice; i++)
    {
        sommeSignature += m_bdObjectVector.at(i).getSize();
    }
    indiceCompteur.push_back(sommeSignature - m_bdObjectVector.at(p_indice).getSize());
    indiceCompteur.push_back(sommeSignature - 1);
    return indiceCompteur;
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
  A function use in to update the attribut m_highest_index.  Which is the next name to use.
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
    bool fileTransform = false;



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

    path = m_pathTransform;
    path /= p_fileName;
    path.replace_extension(".txt");
    fileTransform = boost::filesystem3::exists(path);


    return fileCvfh and filePointCloud and filePoseArm and filePoseObject and fileTransform;
}

/*
  Function to load a file.  Will look if the file exist and if the file is already loaded.
  Will update the signature cloud and the object vector
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
            std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > tf = loadTransform(p_fileName);
            ObjectBd returnedObject(p_fileName,
                                    signature_ptr,
                                    cloud_ptr,
                                    arm,
                                    object,
                                    tf);

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
  Return a vector with the pose in it.
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
  Return a vector with the pose in it.
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
  Load a txt file that contain the the transform.
  param[in] the file name.
  Return a vector with the transform in it.
  */

std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > FileAPI::loadTransform(const string &p_filename)
{
    boost::filesystem3::path path(m_pathTransform);
    path /= p_filename;
    path.replace_extension(".txt");

    std::ifstream ifs(path.c_str());
    std::string line;
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > returnVector;
    while(getline(ifs, line))
    {
        std::vector<std::string> splitterVector;
        boost::algorithm::split(splitterVector, line, boost::algorithm::is_any_of(";"));
        std::vector<float> floatVector;
        for(int i = 0; i < splitterVector.size(); i++)
        {
            floatVector.push_back(atof(splitterVector.at(i).c_str()));
        }
        if(floatVector.size() == 16)
        {
            Eigen::Matrix4f matrix;
            matrix << floatVector[0], floatVector[1], floatVector[2], floatVector[3],
                    floatVector[4], floatVector[5], floatVector[6], floatVector[7],
                    floatVector[8], floatVector[9], floatVector[10], floatVector[11],
                    floatVector[12], floatVector[13] ,floatVector[14] ,floatVector[15];
            returnVector.push_back(matrix);
        }
        else
        {
            throw(std::out_of_range("Error while loading the file, not enought coordinate"));
        }
    }
    return returnVector;
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
