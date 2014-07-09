#include <fileAPI.h>
#include <objectBd.h>

int main(int argc, char** argv)
{

    FileAPI bdApi("/home/jeanjean/rosWorkspace/src/perception3d/dataSet_bd");

    pcl::PointCloud<pcl::VFHSignature308>::Ptr signature(new pcl::PointCloud<pcl::VFHSignature308>);
    pcl::io::loadPCDFile("/home/jeanjean/rosWorkspace/src/perception3d/dataSet_bd/cvfh/1.pcd", *signature);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/jeanjean/rosWorkspace/src/perception3d/dataSet_bd/pointCloud/1.pcd",*cloud);
    std::vector<tf::Transform> arm;
    std::vector<tf::Transform> object;

    tf::Transform tf;
    tf::Vector3 vec(0, 0, 0);
    tf.setOrigin(vec);
    tf::Quaternion quat;
    quat.setRPY(0, 0, 0);
    tf.setRotation(quat);

    arm.push_back(tf);
    object.push_back(tf);



    ObjectBd test = bdApi.createObject(signature,
                       cloud,
                       arm,
                       object);

    bdApi.saveObject(test);





    return 0;
}

