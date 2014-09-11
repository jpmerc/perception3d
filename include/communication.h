#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <objectExtractor.h>
#include <stdlib.h>
#include <fileAPI.h>
#include <jaco_custom.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>
#include <vtkRenderWindow.h>




class Communication
{
public:

    Communication(ObjectExtractor *p_obj_e, FileAPI *p_api, JacoCustom *p_jaco);
    void callback_android_listener(const std_msgs::String& p_input);
    void coordinate_processing(std_msgs::String p_coordinate);
    void grasp_processing(std_msgs::String p_grasp);
    void train_processing(std_msgs::String p_train);
    void spin_once();

    bool get_coordinate_received() const;
    bool get_grasp_received() const;
    bool get_train_received() const;

    void train(bool saveJacoPose, bool viewTF);
    void repeat();

    void testTFandSurfaceTransforms();
    tf::Transform tfFromEigen(Eigen::Matrix4f trans);

    void fillUserInterfaceWithObjectInfo();


    void testRecognition();
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> recognitionViewer;


private:
    boost::mutex viewer_mutex;

    ObjectExtractor* m_object_ex_ptr;
    FileAPI* m_api_ptr;
    JacoCustom* m_jaco_ptr;

    float m_coordinate_user_sended[2];
    int m_position_vector_cvfh;

    bool m_coordinate_received;
    bool m_grasp_received;
    bool m_train_received;

    tf::Transform m_relative_pose;
    void publishRelativePoseTF(tf::Transform arm, tf::Transform scene, tf::Transform model, tf::Transform final, tf::Transform object_centroid);
    bool m_publish_relative_pose;

    int selected_object_index;
    pcl::PointCloud<PointT>::Ptr selected_pointcloud;
    Eigen::Matrix4f calculated_object_transform;
    int grasp_list_index;

    boost::thread saveToDBWithoutArmPoseThread;
    void saveToDBWithoutArmPose();


};

#endif
