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


#include <pcl/surface/mls.h>

using namespace std;

int l_count = 0;
bool sData = true;
bool tData = true;
pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);

vector<tf::Transform> transforms;

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));



void printToPCLViewer2(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PointT> in(input_cloud,255,0,0);
    pclViewer->addPointCloud<PointT>(input_cloud,in,"source");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> up(input_cloud,0,255,0);
    pclViewer->addPointCloud<PointT>(transformed_cloud,up,"upsampled");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "upsampled");

}

void keyboardEventOccurred2 (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    l_count = l_count + 1;
    if(l_count < 2){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);


        if(event.getKeySym () == "s" && sData){
            viewer->removePointCloud("source");
            sData = false;
        }
        else if(event.getKeySym () == "s" && !sData){
            viewer->removePointCloud("source");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> in(input_cloud,255,0,0);
            viewer->addPointCloud<PointT>(input_cloud,in,"source");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
            sData = true;
        }

        else if(event.getKeySym () == "t" && tData){
            viewer->removePointCloud("upsampled");
            tData = false;
        }
        else if(event.getKeySym () == "t" && !tData){
            viewer->removePointCloud("upsampled");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> up(input_cloud,0,255,0);
            viewer->addPointCloud<PointT>(transformed_cloud,up,"upsampled");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "upsampled");
            tData = true;
        }


    }
    else{
        l_count = 0;
    }

}


pcl::PointCloud<PointT>::Ptr MovingLeastSquaresUpsampling(pcl::PointCloud<PointT>::Ptr cloud){

    // Surface Approximation
    ros::Time begin = ros::Time::now();
    pcl::MovingLeastSquares<PointT, PointT> mls;
    pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
    mls.setInputCloud (cloud);
    mls.setSearchRadius (0.04);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.process (*cloud_smoothed);
    ros::Time end = ros::Time::now();
    std::cout << "Surface Approximation Time " << end - begin << " seconds" << std::endl;




    // Upsampling
    begin = ros::Time::now();
    pcl::PointCloud<PointT>::Ptr cloud_smoothed2(new pcl::PointCloud<PointT>);
    pcl::MovingLeastSquares<PointT, PointT> mls2;
    mls2.setInputCloud (cloud_smoothed);
    mls2.setSearchRadius (0.02);
    mls2.setPolynomialFit (true);
    mls2.setPolynomialOrder (2);
    mls2.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
    mls2.setUpsamplingRadius (0.01);
    mls2.setUpsamplingStepSize (0.005);
    mls2.process (*cloud_smoothed2);
    end = ros::Time::now();
    std::cout << "Upsampling Time " << end - begin << " seconds" << std::endl;

    begin = ros::Time::now();
    pcl::VoxelGrid<PointT> vx_grid;
    vx_grid.setInputCloud (cloud_smoothed2);

    // 0.1 to see correctly in viewer, 0.005 to do it like in the main program
    double leaf_size = 0.01;
    vx_grid.setLeafSize (leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    vx_grid.filter (*cloud_filtered);
    end = ros::Time::now();
    std::cout << "Voxel Grid Time " << end - begin << " seconds" << std::endl;

    pcl::io::savePCDFileBinary(string("/home/jp/devel/src/perception3d/screenshots/upsampling.pcd"),*cloud_filtered);
    return cloud_filtered;
}

int main (int argc, char** argv){

    // Initialize ROS
    ros::init (argc, argv, "snapshot");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/screenshots/test_kleenex_translation4.pcd", *input_cloud);
    //pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/screenshots/test_stapler.pcd", *input_cloud);

    MovingLeastSquaresUpsampling(input_cloud);
    sleep(1);

    pcl::PointCloud<PointT>::Ptr test(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/screenshots/upsampling.pcd", *transformed_cloud);

    //pcl::transformPointCloud(*input_cloud,*transformed_cloud,sgurf_tf.at(0));

    //PCL Viewer
    pclViewer->registerKeyboardCallback (keyboardEventOccurred2, (void*)&pclViewer);
    pclViewer->setBackgroundColor (0, 0, 0);
    pclViewer->initCameraParameters ();
    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(800,450);
    renderWindow->Render();

    printToPCLViewer2();

    ros::Rate r(30);
    tf::TransformBroadcaster br;

    while (ros::ok() && !pclViewer->wasStopped()) {
        pclViewer->spinOnce (100);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
