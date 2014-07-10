#include <objectBd.h>


ObjectBd::ObjectBd(std::string p_name,
                   pcl::PointCloud<pcl::VFHSignature308>::Ptr p_signature,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pointCloud,
                   std::vector<tf::Transform> p_armPose,
                   std::vector<tf::Transform> p_objectPose):
    m_name(p_name),
    m_relative_arm_pose(p_armPose),
    m_object_pose(p_objectPose)
{
    m_pcSize = p_signature->size();
    m_object_signature = p_signature;
    m_object_point_cloud = p_pointCloud;
    m_fullLoaded = true;
}

ObjectBd::ObjectBd(std::string p_name,
                   pcl::PointCloud<pcl::VFHSignature308>::Ptr p_signature):
    m_name(p_name)
{
    m_pcSize = p_signature->size();
    m_object_signature = p_signature;
    m_fullLoaded = false;
}

bool ObjectBd::setAllAttribut(std::string p_name,
                              pcl::PointCloud<pcl::VFHSignature308>::Ptr p_signature,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pointCloud,
                              std::vector<tf::Transform> p_armPose,
                              std::vector<tf::Transform> p_objectPose)
{
    m_name = p_name;
    m_pcSize = p_signature->size();
    m_object_signature = p_signature;
    m_object_point_cloud = p_pointCloud;
    m_object_pose = p_objectPose;
    m_relative_arm_pose = p_armPose;
    m_fullLoaded = true;
}

std::vector<tf::Transform> ObjectBd::getArmPose() const
{
    if(m_fullLoaded)
    {
        return m_relative_arm_pose;
    }

}

std::vector<tf::Transform> ObjectBd::getObjectPose() const
{
    if(m_fullLoaded)
    {
        return m_object_pose;
    }

}

pcl::PointCloud<pcl::VFHSignature308>::Ptr ObjectBd::getSignature() const
{
    return m_object_signature;

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectBd::getPointCloud() const
{
    if(m_fullLoaded)
    {
        return m_object_point_cloud;
    }

}

std::string ObjectBd::getName() const
{
    return m_name;

}

int ObjectBd::getSize() const
{
    return m_pcSize;
}

bool ObjectBd::objectIsComplete() const
{
    return m_fullLoaded;
}

ObjectBd& ObjectBd::operator =(const ObjectBd& p_object)
{
    m_name = p_object.getName();
    *m_object_signature = *p_object.getSignature();
    *m_object_point_cloud = *p_object.getPointCloud();
    for(int i = 0; i < p_object.getArmPose().size(); i++)
    {
        m_relative_arm_pose.push_back(p_object.getArmPose().at(i));
    }
    for(int i = 0; i = p_object.getObjectPose().size(); i++)
    {
        m_object_pose.push_back(p_object.getObjectPose().at(i));
    }
    m_fullLoaded = p_object.objectIsComplete();
    m_pcSize = p_object.getSize();

    return *this;
}

bool ObjectBd::operator ==(const ObjectBd& p_object)
{
    bool validation = true;

    validation = m_name == p_object.getName();
    if(validation)
        validation = m_pcSize == p_object.getSize();
    if(validation)
    {
        for(int i = 0; i < m_object_signature->size(); i++)
        {
            for(int j = 0; j < 308; j++)
            {
                if(m_object_signature->at(i).histogram[j] != p_object.getSignature()->at(i).histogram[j])
                {
                    validation = false;
                }
            }
        }
    }
    return validation;

}
