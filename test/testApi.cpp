#include <fileAPI.h>
#include <objectBd.h>
#include <gtest/gtest.h>

class FileApiTest : public testing::Test
{
public:
    FileApiTest():m_bd((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd").c_str())
    {
        m_signatureLoaded.reset(new pcl::PointCloud<pcl::VFHSignature308>);
        m_pointCloudLoadrd.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    void loadCloud(const std::string& p_filename)
    {
        boost::filesystem3::path path((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd").c_str());
        boost::filesystem3::path pathTemp;
        pathTemp = path;
        pathTemp /= "cvfh/" + p_filename;
        pathTemp.replace_extension(".pcd");
        pcl::io::loadPCDFile(pathTemp.c_str(), *m_signatureLoaded);
        pathTemp = path;
        pathTemp  /= "pointCloud/" + p_filename;
        pathTemp.replace_extension(".pcd");
        pcl::io::loadPCDFile(pathTemp.c_str(), *m_pointCloudLoadrd);
    }

    FileAPI m_bd;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr m_signatureLoaded;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloudLoadrd;
};

class BdCleaner : public testing::Test
{
public:
    BdCleaner()
    {
        boost::filesystem3::path path((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd").c_str());
        boost::filesystem3::recursive_directory_iterator dirIt(path);
        boost::filesystem3::path tempPath;
        while(dirIt != boost::filesystem3::recursive_directory_iterator())
        {
            tempPath = * dirIt;
            if(tempPath.stem() == "5")
            {
                boost::filesystem3::remove(tempPath);
            }
            dirIt++;
        }
    }
};


    TEST(FileAPI, Default)
    {
        boost::filesystem3::path path((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd").c_str());
        FileAPI fileBd(path.c_str());
        ASSERT_EQ(fileBd.getAllHistograme()->size(), 4);
        ASSERT_EQ(fileBd.getAllObjects().size(), 4);
    }

    TEST(FileAPI, BadPath)
    {
        boost::filesystem3::path path("/home/robot/rosWorspace/src/perception");
        ASSERT_THROW(FileAPI fileBd(path.c_str()), std::runtime_error);
        boost::filesystem3::path path2("daskldj");
        ASSERT_THROW(FileAPI fileBd(path2.c_str()), std::runtime_error);
        boost::filesystem3::path path3("/home/robot/rosWorkspace/src/perception3d/");
        ASSERT_THROW(FileAPI fileBd(path3.c_str()), std::runtime_error);
    }

    TEST_F(FileApiTest, Getter)
    {
        ASSERT_EQ(m_bd.getAllHistograme()->size(),4);
        ASSERT_EQ(m_bd.getAllObjects().size(),4);
        ASSERT_THROW(m_bd.getObjectByIndex(10), std::out_of_range);
        ASSERT_THROW(m_bd.getHistogrameByIndex(5), std::out_of_range);
        ASSERT_NO_THROW(m_bd.getHistogrameByIndex(0));
        ASSERT_NO_THROW(m_bd.getObjectByIndex(3));
    }

    TEST_F(FileApiTest, CreateObject)
    {
        pcl::PointCloud<pcl::VFHSignature308>::Ptr signature(new pcl::PointCloud<pcl::VFHSignature308>);
        pcl::io::loadPCDFile((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd/cvfh/1.pcd").c_str(), *signature);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd/pointCloud/1.pcd").c_str(), *cloud);

        tf::Vector3 vec(0,0,0);

        tf::Transform arm;
        arm.setOrigin(vec);
        arm.getBasis().setEulerYPR(0,0,0);

        tf::Transform object;
        object.setOrigin(vec);
        object.getBasis().setEulerYPR(0,0,0);

        std::vector<tf::Transform> armVec;
        std::vector<tf::Transform> objectVec;
        armVec.push_back(arm);
        objectVec.push_back(object);

        ObjectBd testObj = m_bd.createObject(signature,
                                             cloud,
                                             armVec,
                                             objectVec);

        ASSERT_TRUE(testObj.getName() == "5");
        ASSERT_TRUE(testObj.getSize() == signature->size());
        for(int i = 0; i < 308; i++)
        {
            ASSERT_TRUE(testObj.getSignature()->at(0).histogram[i] == signature->at(0).histogram[i]);
        }

        signature.reset(new pcl::PointCloud<pcl::VFHSignature308>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        ASSERT_TRUE(testObj.getSignature()->size() == 1);

        ASSERT_THROW(m_bd.createObject(signature,
                                       cloud,
                                       armVec,
                                       objectVec),std::runtime_error);

    }

    TEST_F(FileApiTest, SaveFile)
    {
        pcl::PointCloud<pcl::VFHSignature308>::Ptr signature(new pcl::PointCloud<pcl::VFHSignature308>);
        pcl::io::loadPCDFile((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd/cvfh/1.pcd").c_str(), *signature);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd/pointCloud/1.pcd").c_str(), *cloud);

        tf::Vector3 vec(0,0,0);
        tf::Quaternion quat;
        quat.setRPY(0,0,0);

        tf::Transform arm;
        arm.setOrigin(vec);
        arm.setRotation(quat);

        tf::Transform object;
        object.setOrigin(vec);
        object.setRotation(quat);

        std::vector<tf::Transform> armVec;
        std::vector<tf::Transform> objectVec;
        armVec.push_back(arm);
        objectVec.push_back(object);

        m_bd.save(signature,
                  cloud,
                  armVec,
                  objectVec);

        boost::filesystem3::path path((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd").c_str());
        boost::filesystem3::recursive_directory_iterator dirIt(path);
        boost::filesystem3::path pathTemp;

        int compteurFichier = 0;
        pathTemp = path;
        pathTemp /= "cvfh/5.pcd";
        if(boost::filesystem3::exists(pathTemp))
        {
            compteurFichier++;
        }
        pathTemp = path;
        pathTemp /= "pointCloud/5.pcd";
        if(boost::filesystem3::exists(pathTemp))
        {
            compteurFichier++;
        }
        pathTemp = path;
        pathTemp /= "poseArm/5.txt";
        if(boost::filesystem3::exists(pathTemp))
        {
            compteurFichier++;
        }
        pathTemp = path;
        pathTemp /= "poseObject/5.txt";
        if(boost::filesystem3::exists(pathTemp))
        {
            compteurFichier++;
        }
        ASSERT_EQ(compteurFichier,4);


        signature.reset(new pcl::PointCloud<pcl::VFHSignature308>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        armVec.clear();
        objectVec.clear();

        ASSERT_THROW(m_bd.save(signature,
                                cloud,
                                armVec,
                                objectVec), std::runtime_error);
    }

    TEST_F(FileApiTest, SaveAgain)
    {
        //ASSERT_NO_THROW(ObjectBd testObj = m_bd.loadFile("5"));
        ObjectBd test = m_bd.getObjectByIndex(4);


        m_bd.saveObject(test);

        std::ifstream ifs((boost::filesystem3::current_path() /= "../../../src/perception3d/dataSet_bd/poseObject/5.txt").c_str());
        std::string line;
        int compteurLine = 0;
        while(std::getline(ifs, line))
        {
            compteurLine++;
        }
        ifs.close();
        ASSERT_EQ(compteurLine,2);
    }

    TEST_F(FileApiTest, LoadTwice)
    {
        ASSERT_NO_THROW(m_bd.loadFile("1"));
    }

    TEST_F(FileApiTest , SimilarObject)
    {
        ObjectBd obj1 = m_bd.getObjectByIndex(1);
        ObjectBd obj2 = m_bd.getObjectByIndex(1);
        ObjectBd obj3 = m_bd.getObjectByIndex(4);

        ASSERT_TRUE(obj1 == obj2);
        ASSERT_TRUE(obj1 == obj1);
        ASSERT_TRUE(obj3 == obj3);
        ASSERT_FALSE(obj1 == obj3);
        ASSERT_FALSE(obj2 == obj3);
    }


    TEST_F(FileApiTest, GetGoodObject)
    {
        loadCloud("1");

        ObjectBd objRetrived = m_bd.retrieveObjectFromHistogramme(0);
    }

    TEST_F(BdCleaner, BdCleanAfterTest)
    {
        std::cout << "BD clean" << std::endl;
    }



