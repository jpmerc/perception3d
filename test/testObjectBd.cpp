#include <gtest/gtest.h>
#include <objectBd.h>

class ObjectBdTest : public testing::Test
{
public:
    ObjectBdTest(){}

    void makeDate(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_signature,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud,
                  std::vector<tf::Transform> p_armVec,
                  std::vector<tf::Transform> p_objectVec)
    {
        pcl::VFHSignature308 vfh;
        for(int i = 0; i < 308; i++)
        {
            vfh.histogram[i] = 0;
        }
        p_signature->push_back(vfh);

        pcl::PointXYZRGB point(255,0,0);
        for(int i = 0; i < 11; i++)
        {
            point.x = i;
            point.y = i;
            point.z = i;
            p_cloud->push_back(point);
        }

        tf::Vector3 vec(0,0,0);
        tf::Quaternion quat;
        quat.setRPY(0,0,0);

        tf::Transform arm;
        tf::Transform object;

        arm.setOrigin(vec);
        arm.setRotation(quat);
        object.setOrigin(vec);
        object.setRotation(quat);

        p_armVec.push_back(arm);
        p_objectVec.push_back(object);
    }
};


TEST_F(ObjectBdTest, Constructor)
{
    pcl::PointCloud<pcl::VFHSignature308>::Ptr signature_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<tf::Transform> arm;
    std::vector<tf::Transform> object;

    makeDate(signature_ptr,
             cloud_ptr,
             arm,
             object);

    ASSERT_NO_THROW(ObjectBd test(std::string("1"),
                                    signature_ptr,
                                  cloud_ptr,
                                  arm,
                                  object));


}

