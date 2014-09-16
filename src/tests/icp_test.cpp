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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

vector<tf::Transform> transforms;

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > sgurf_tf;
std::vector<Eigen::Vector3f> centroids;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> surfaces_pc;


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

    }
    else{
        l_count = 0;
    }

}


void printToPCLViewer2(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PointT> in(input_cloud,255,0,0);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(input_cloud,in,"source");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
    Eigen::Vector4f c;
    pcl::compute3DCentroid<PointT>(*input_cloud,c);
    cout << "Source : [" << c[0] << " " << c[1] << " " << c[2] << "]" << endl;

    for(int i=0; i < surfaces_pc.size(); i++){
        pcl::PointCloud<PointT>::Ptr pc = surfaces_pc[i];
        pcl::visualization::PointCloudColorHandlerRandom<PointT> randColor(pc);
        std::stringstream ss;
        ss << i;
        std::string ind = ss.str();
        std::string pc_name = "surface_" + ind;
        pclViewer->addPointCloud<PointT>(pc,randColor,pc_name);
        pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pc_name);
    }


//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> centroid_color (centroid_pointcloud, 255, 0, 255);
//    pclViewer->addPointCloud<pcl::PointXYZ> (centroid_pointcloud, centroid_color, "centroids");
//    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 12, "centroids");

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
            viewer->addPointCloud<pcl::PointXYZRGB>(input_cloud,in,"source");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
            sData = true;
        }


    }
    else{
        l_count = 0;
    }

}




int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "snapshot");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/screenshots/test_kleenex_translation.pcd", *input_cloud);


    FileAPI *fileAPI = new FileAPI(string("/home/jp/devel/src/perception3d/database"));
    Object_recognition *Recogn = new Object_recognition();

    tf::Transform noMovement;
    noMovement.setIdentity();
    //noMovement.setOrigin(tf::Vector3(0.02,-0.04,0.03));
    //noMovement.setRotation(tf::Quaternion(angles::from_degrees(8),angles::from_degrees(-8),angles::from_degrees(12)));
    Eigen::Matrix4f transformMatrix;
    pcl_ros::transformAsMatrix(noMovement,transformMatrix);

    //Eigen::Matrix4f transformMatrix;ss

//    int selected_object_index = Recogn->OURCVFHRecognition(input_cloud, fileAPI, transformMatrix, transforms);


    Recogn->makeCVFH(input_cloud, sgurf_tf, centroids, surfaces_pc);
    for(int i=0; i < centroids.size(); i++){
        pcl::PointXYZ pt;
        pt.x = centroids.at(i)[0];
        pt.y = centroids.at(i)[1];
        pt.z = centroids.at(i)[2];
        centroid_pointcloud->push_back(pt);
    }



//    ObjectBd obj = fileAPI->retrieveObjectFromHistogram(selected_object_index);
//    model_cloud = obj.getPointCloud();

//    pcl::transformPointCloud(*input_cloud,*transformed_cloud,transformMatrix);

    //PCL Viewer
    pclViewer->registerKeyboardCallback (keyboardEventOccurred2, (void*)&pclViewer);
    pclViewer->setBackgroundColor (0, 0, 0);
    pclViewer->initCameraParameters ();
    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(800,450);
    renderWindow->Render();

    printToPCLViewer2();



    tf::TransformBroadcaster br;



    ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped()) {
        pclViewer->spinOnce (100);
        ros::spinOnce();
        r.sleep();

//        br.sendTransform(tf::StampedTransform(transforms.at(0),ros::Time::now(),"camera_rgb_frame","object_source_tf"));
//        br.sendTransform(tf::StampedTransform(transforms.at(1),ros::Time::now(),"camera_rgb_frame","object_model_tf"));
//        br.sendTransform(tf::StampedTransform(transforms.at(2),ros::Time::now(),"object_source_tf","object_transform_tf"));

    }
    return 0;
}
