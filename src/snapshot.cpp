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

std::string base_filename;
std::string filename;
std::string directory;
int NumberOfSnapshots = 0;
int l_count = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));


void updateFilename(int number){
    char new_filename[250];
    if(NumberOfSnapshots > 0){
        sprintf(new_filename,"%s%d.pcd",base_filename.c_str(),number+1);
    }
    else{
        sprintf(new_filename,"%s.pcd",base_filename.c_str());
    }
    filename = std::string(new_filename);
}

void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"source cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
}

void cloud_callback (const pcl::PCLPointCloud2ConstPtr& input){
    pcl::fromPCLPointCloud2(*input,*cloud);
    printToPCLViewer();
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    l_count = l_count + 1;
    if(l_count < 2){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym () == "s"){
            if(!cloud->empty()){
                std::string path = directory + filename;
                pcl::io::savePCDFileASCII(path,*cloud);
                std::cout << "File saved to " << path << std::endl;

                //Increment index at the end of filename
                NumberOfSnapshots++;
                updateFilename(NumberOfSnapshots);
            }
            else{
                std::cerr << "No pointcloud to save!" << endl;
            }
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

    //Default path to save pcd files
    char dir[250];
    sprintf(dir,"%s/.ros/snapshots/",getenv("HOME"));
    n.param("directory",directory,std::string(dir));
    n.param("filename",base_filename, std::string("snapshots"));
    std::cout << "Directory =  " << directory << std::endl;
    std::cout << "filename =  " << base_filename << std::endl;


    //Create directory if it does not exist
    if (!boost::filesystem3::exists(directory)){
        boost::filesystem3::create_directory(directory);
    }

    //Count the number of files of the same name
    int NumberOfFilesInDir = std::distance(boost::filesystem3::directory_iterator(directory), boost::filesystem3::directory_iterator());
    boost::filesystem3::directory_iterator it = boost::filesystem3::directory_iterator(directory);
    int highestIndex = 0;
    for(int i=0; i<NumberOfFilesInDir; i++){
        std::string temp_filename = boost::filesystem3::basename(*it);
        if(temp_filename.find(base_filename) != std::string::npos){
            std::string ind = temp_filename.substr(base_filename.length());
            int index = atoi(ind.c_str());
            if(index > highestIndex){
                highestIndex = index;
            }
        }
        if(i < (NumberOfFilesInDir-1)){ //condition to avoid pointing at nothing
            it++;
        }
    }

    NumberOfSnapshots = highestIndex;
    updateFilename(NumberOfSnapshots);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_callback);

    //PCL Viewer
    pclViewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&pclViewer);
    pclViewer->setBackgroundColor (0, 0, 0);
    pclViewer->initCameraParameters ();
    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(800,450);
    renderWindow->Render();

    ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped()) {
        pclViewer->spinOnce (100);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
