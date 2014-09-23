#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>

#include <stdlib.h>
#include <sstream>
#include <stdio.h>

#include "boost/filesystem.hpp"

#include <fileAPI.h>
#include <object_recognition.h>

using namespace std;

std::string base_filename;
std::string filename;
std::string directory;
int NumberOfSnapshots = 0;
int l_count = 0;
bool sData = true;
bool tData = true;
bool mData = true;
bool vData = true;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelized_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

vector<tf::Transform> transforms;

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));



void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PointT> in(input_cloud,255,0,0);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(input_cloud,in,"source");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
    Eigen::Vector4f c;
    pcl::compute3DCentroid<PointT>(*input_cloud,c);
    cout << "Source : [" << c[0] << " " << c[1] << " " << c[2] << "]" << endl;

    pcl::visualization::PointCloudColorHandlerCustom<PointT> model(model_cloud,0,255,0);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(model_cloud,model,"model");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model");
    Eigen::Vector4f c2;
    pcl::compute3DCentroid<PointT>(*model_cloud,c2);
    cout << "Model : [" << c2[0] << " " << c2[1] << " " << c2[2] << "]" << endl;


    pcl::visualization::PointCloudColorHandlerCustom<PointT> tf(transformed_cloud,0,0,255);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(transformed_cloud,tf,"tf");
    pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tf");
    Eigen::Vector4f c3;
    pcl::compute3DCentroid<PointT>(*transformed_cloud,c3);
    cout << "Transform : [" << c3[0] << " " << c3[1] << " " << c3[2] << "]" << endl;


//    pcl::visualization::PointCloudColorHandlerCustom<PointT> vox(voxelized_cloud,110,70,175);
//    pclViewer->addPointCloud<pcl::PointXYZRGB>(voxelized_cloud,vox,"voxel");
//    pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "voxel");
//    Eigen::Vector4f c4;
//    pcl::compute3DCentroid<PointT>(*transformed_cloud,c4);
//    cout << "Voxelized : [" << c4[0] << " " << c4[1] << " " << c4[2] << "]" << endl;
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    l_count = l_count + 1;
    if(l_count < 2){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym () == "t" && tData){
            viewer->removePointCloud("tf");
            tData = false;
        }
        else if (event.getKeySym () == "t" && !tData){
            pcl::visualization::PointCloudColorHandlerCustom<PointT> tf(transformed_cloud,0,0,255);
            viewer->addPointCloud<pcl::PointXYZRGB>(transformed_cloud,tf,"tf");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tf");
            tData = true;
        }

        else if(event.getKeySym () == "s" && sData){
            viewer->removePointCloud("source");
            sData = false;
        }
        else if(event.getKeySym () == "s" && !sData){
            viewer->removePointCloud("source");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> in(input_cloud,255,0,0);
            viewer->addPointCloud<pcl::PointXYZRGB>(input_cloud,in,"source");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
            sData = true;
        }
        else if(event.getKeySym () == "m" && mData){
            viewer->removePointCloud("model");
            mData = false;
        }
        else if(event.getKeySym () == "m" && !mData){
            viewer->removePointCloud("model");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> model(model_cloud,0,255,0);
            viewer->addPointCloud<pcl::PointXYZRGB>(model_cloud,model,"model");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model");
            mData = true;
        }

//        else if(event.getKeySym () == "v" && vData){
//            viewer->removePointCloud("voxel");
//            vData = false;
//        }
//        else if(event.getKeySym () == "v" && !vData){
//            viewer->removePointCloud("voxel");
//            pcl::visualization::PointCloudColorHandlerCustom<PointT> vox(voxelized_cloud,110,70,175);
//            pclViewer->addPointCloud<pcl::PointXYZRGB>(voxelized_cloud,vox,"voxel");
//            pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "voxel");
//            vData = true;
//        }

    }
    else{
        l_count = 0;
    }

}


int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "snapshot");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/screenshots/test_kleenex_translation2.pcd", *input_cloud);


    FileAPI *fileAPI = new FileAPI(string("/home/jp/devel/src/perception3d/database"));
    Object_recognition *Recogn = new Object_recognition();

   // tf::Transform noMovement;
   // noMovement.setIdentity();
    //noMovement.setOrigin(tf::Vector3(0.02,-0.04,0.03));
    //noMovement.setRotation(tf::Quaternion(angles::from_degrees(8),angles::from_degrees(-8),angles::from_degrees(12)));
    Eigen::Matrix4f transformMatrix;
   // pcl_ros::transformAsMatrix(noMovement,transformMatrix);

    //Eigen::Matrix4f transformMatrix;

    int selected_object_index = Recogn->OURCVFHRecognition(input_cloud, fileAPI, transformMatrix, transforms);

    ObjectBd obj = fileAPI->retrieveObjectFromHistogram(selected_object_index);
    model_cloud = obj.getPointCloud();

    pcl::transformPointCloud(*input_cloud,*transformed_cloud,transformMatrix);

    //voxelized_cloud = Recogn->transformAndVoxelizePointCloud(input_cloud,model_cloud,transformMatrix);

    //PCL Viewer
    pclViewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&pclViewer);
    pclViewer->setBackgroundColor (0, 0, 0);
    pclViewer->initCameraParameters ();
    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(800,450);
    renderWindow->Render();

    printToPCLViewer();

    tf::TransformBroadcaster br;

    ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped()) {
        pclViewer->spinOnce (100);
        ros::spinOnce();
        r.sleep();
        if(transforms.size() >= 3){

            Eigen::Matrix4f src,target,transform;
            pcl_ros::transformAsMatrix(transforms.at(0), src);
            pcl_ros::transformAsMatrix(transforms.at(1), target);
            pcl_ros::transformAsMatrix(transforms.at(2), transform);


            tf::Transform kinect_src = Recogn->transformKinectFrameToWorldFrame(transforms.at(0));
            tf::Transform kinect_model= Recogn->transformKinectFrameToWorldFrame(transforms.at(1));
            tf::Transform kinect_transform = Recogn->transformKinectFrameToWorldFrame(transforms.at(0) * transforms.at(2));

            Eigen::Matrix4f src2,target2,transform2;
            pcl_ros::transformAsMatrix(kinect_src, src2);
            pcl_ros::transformAsMatrix(kinect_model, target2);
            pcl_ros::transformAsMatrix(kinect_transform, transform2);


//            cout << "Source_RGB : "     << endl <<  src         << endl;
//            cout << "Model_RGB : "      << endl <<  target      << endl;
//            cout << "Transform_RGB : "  << endl <<  transform   << endl;

//            cout << "Source_WORLD : "     << endl <<  src2         << endl;
//            cout << "Model_WORLD : "      << endl <<  target2      << endl;
//            cout << "Transform_WORLD : "  << endl <<  transform2   << endl;


            tf::Transform good_tf = kinect_src.inverse() * kinect_model;
            Eigen::Matrix4f good_matrix;
            pcl_ros::transformAsMatrix(good_tf, good_matrix);
       //     cout << "Transform_WORLD_GOOD : "  << endl <<  good_matrix   << endl;

            Eigen::Matrix4f t; t.setZero(); t(0,2)=1; t(1,0)=-1; t(2,1)=-1; t(3,3)=1;
            Eigen::Vector4f vec1(src(0,3), src(1,3), src(2,3), 1);
            Eigen::Vector4f vec2 = vec1 + Eigen::Vector4f(transform(0,3), transform(1,3), transform(2,3), 0);

            Eigen::Vector4f res1 = t * vec1;
            Eigen::Vector4f res2 = t * vec2;
            Eigen::Vector4f transl = res2 - res1;

            tf::Vector3 vec_tf(transl(0),transl(1),transl(2));

            tf::Transform test_tf;
            test_tf.setIdentity();
            test_tf.setOrigin(vec_tf);


            br.sendTransform(tf::StampedTransform(kinect_src,ros::Time::now(),"camera_rgb_frame","object_source_tf"));
            br.sendTransform(tf::StampedTransform(kinect_model,ros::Time::now(),"camera_rgb_frame","object_model_tf"));
            br.sendTransform(tf::StampedTransform(kinect_transform,ros::Time::now(),"camera_rgb_frame","object_transform_tf"));
       //     br.sendTransform(tf::StampedTransform(transforms.at(0),ros::Time::now(),"camera_rgb_frame","object_source_tf"));
       //     br.sendTransform(tf::StampedTransform(transforms.at(1),ros::Time::now(),"camera_rgb_frame","object_model_tf"));
       //    br.sendTransform(tf::StampedTransform(transforms.at(2),ros::Time::now(),"object_source_tf","object_transform_tf"));
            br.sendTransform(tf::StampedTransform(good_tf,ros::Time::now(),"object_source_tf","object_transform_tf_good"));
            br.sendTransform(tf::StampedTransform(test_tf,ros::Time::now(),"object_source_tf","object_transform_tf_TEST"));
        }
    }
    return 0;
}
