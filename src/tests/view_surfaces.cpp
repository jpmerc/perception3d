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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr sgurf_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

vector<tf::Transform> transforms;

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > sgurf_tf;
std::vector<Eigen::Vector3f> centroids;
std::vector<Eigen::Vector3f> normals;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> surfaces_pc;

std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > sgurf_tf2;
std::vector<Eigen::Vector3f> centroids2;
std::vector<Eigen::Vector3f> normals2;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> surfaces_pc2;


void printToPCLViewer2(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PointT> in(input_cloud,255,0,0);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(input_cloud,in,"source");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
    Eigen::Vector4f c;
    pcl::compute3DCentroid<PointT>(*input_cloud,c);
    cout << "Source Centroid : [" << c[0] << " " << c[1] << " " << c[2] << "]" << endl;

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


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> centroid_color (centroid_pointcloud, 255, 0, 255);
    pclViewer->addPointCloud<pcl::PointXYZ> (centroid_pointcloud, centroid_color, "centroids");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 12, "centroids");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sgurf_color (sgurf_pointcloud, 200, 110, 125);
    pclViewer->addPointCloud<pcl::PointXYZ> (sgurf_pointcloud, sgurf_color, "sgurfs");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sgurfs");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> tf(transformed_cloud,0,0,255);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(transformed_cloud,tf,"tf");
    pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tf");

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




int main (int argc, char** argv){

    // Initialize ROS
    ros::init (argc, argv, "snapshot");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/screenshots/test_kleenex_translation4.pcd", *input_cloud);

    Eigen::Matrix4f t; t.setZero(); t(0,0)=1; t(1,1)=1; t(2,2)=1; t(3,3)=1;
    t(0,3)=0.2; // X translation of 20 cm
    pcl::transformPointCloud(*input_cloud,*input_cloud2,t);



    FileAPI *fileAPI = new FileAPI(string("/home/jp/devel/src/perception3d/database"));
    Object_recognition *Recogn = new Object_recognition();


    Recogn->makeCVFH(input_cloud,  sgurf_tf,  centroids,  surfaces_pc,  normals );
    Recogn->makeCVFH(input_cloud2, sgurf_tf2, centroids2, surfaces_pc2, normals2);



    Eigen::Matrix4f translationMatrix = sgurf_tf.at(0).inverse() * sgurf_tf2.at(0);
    cout << endl << translationMatrix.inverse() << endl;






    Eigen::Vector4f c;
    pcl::compute3DCentroid<PointT>(*input_cloud,c);


    for(int i=0; i < centroids.size(); i++){
        pcl::PointXYZ pt;
        Eigen::Vector3f cent = centroids.at(i);
        pt.x = cent(0);
        pt.y = cent(1);
        pt.z = cent(2);
        centroid_pointcloud->push_back(pt);

        Eigen::Matrix4f tf_matrix = sgurf_tf.at(i);
        Eigen::Vector4f c4f(0,0,0,1);
        Eigen::Vector4f v = tf_matrix.inverse() * c4f;
        pt.x = v(0);
        pt.y = v(1);
        pt.z = v(2);
        sgurf_pointcloud->push_back(pt);
    }

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
        for(int i=0; i < sgurf_tf.size(); i++){
            tf::Transform tf_ =  Recogn->tfFromEigen(sgurf_tf.at(i));
            tf_ = Recogn->transformKinectFrameToWorldFrame(tf_);
            stringstream ss;
            ss << i;
            string frame = "SGURF_" + ss.str();
            //cout << "Sending " << frame << endl;
            br.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "camera_rgb_frame", frame));
        }
        r.sleep();
    }
    return 0;
}
