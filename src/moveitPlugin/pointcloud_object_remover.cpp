#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread/mutex.hpp>


using namespace std;

struct BoundingBox{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
};

shape_msgs::SolidPrimitive shape_;
geometry_msgs::Pose pose_;
boost::mutex object_mutex;
BoundingBox findBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc);
const float bad_point = std::numeric_limits<float>::quiet_NaN();
ros::Publisher pc_publisher;
ros::Publisher bb_publisher;
struct BoundingBox myBoundingBox;
void printBoundingBox(BoundingBox in_bb);
void publishBoundingBoxAsPointCloud(BoundingBox in_bb);
int bbox_seq_number = 0;



void object_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc){

    struct BoundingBox temp_BB = findBoundingBox(pc);

    object_mutex.lock();
    myBoundingBox = temp_BB;
    object_mutex.unlock();
}


void scene_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& scene_cloud){

    pcl::PointCloud<pcl::PointXYZRGB> pc = *scene_cloud;
    cout << "Frame_id = " << pc.header.frame_id << endl;

    object_mutex.lock();
    struct BoundingBox BB = myBoundingBox;
    object_mutex.unlock();

    publishBoundingBoxAsPointCloud(BB);
    printBoundingBox(BB);

    for(int i = 0; i < scene_cloud->points.size(); i++){

        double x = pc.points.at(i).x;
        double y = pc.points.at(i).y;
        double z = pc.points.at(i).z;

        if(x >= BB.x_min && x <= BB.x_max && y >= BB.y_min && y <= BB.y_max && z >= BB.x_min && z <= BB.x_max ){
            pc.points.at(i).x =  bad_point;
            pc.points.at(i).y =  bad_point;
            pc.points.at(i).z =  bad_point;
        }
    }

    pc_publisher.publish(pc);

}



// Find a bounding box around the object to grasp (pc)
// Outputs the shape and the pose of the bounding box
// Inspired from https://github.com/unboundedrobotics/ubr1_preview/blob/master/ubr1_grasping/src/shape_extraction.cpp
BoundingBox findBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc){

    struct BoundingBox BB;

    ros::NodeHandle nh;
    ros::Rate r(3);
    double x_min =  9999.0;
    double x_max = -9999.0;
    double y_min =  9999.0;
    double y_max = -9999.0;
    double z_min =  9999.0;
    double z_max = -9999.0;

    for(int i = 0; i < pc->size(); i++){
        double pc_x = pc->at(i).x;
        double pc_y = pc->at(i).y;
        double pc_z = pc->at(i).z;

        if(pc_x < x_min) x_min = pc_x;
        if(pc_y < y_min) y_min = pc_y;
        if(pc_z < z_min) z_min = pc_z;

        if(pc_x > x_max) x_max = pc_x;
        if(pc_y > y_max) y_max = pc_y;
        if(pc_z > z_max) z_max = pc_z;
    }


    BB.x_min = x_min;
    BB.x_max = x_max;
    BB.y_min = y_min;
    BB.y_max = y_max;
    BB.z_min = z_min;
    BB.z_max = z_max;


    return BB;

}

void printBoundingBox(BoundingBox in_bb){

    cout << "X : [" << in_bb.x_min << " -> " << in_bb.x_max << "]" << endl;
    cout << "Y : [" << in_bb.y_min << " -> " << in_bb.y_max << "]" << endl;
    cout << "Z : [" << in_bb.z_min << " -> " << in_bb.z_max << "]" << endl;

}


void publishBoundingBoxAsPointCloud(BoundingBox in_bb){
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pc.header.frame_id = "/camera_rgb_optical_frame";
    pc.header.seq = bbox_seq_number;
    //pc.header.stamp = ros::Time::now().toSec();
    bbox_seq_number++;

//    int NumberOfPointsPerAxis = 10;
//    double x_increment = (in_bb.x_max - in_bb.x_min) / NumberOfPointsPerAxis;
//    double y_increment = (in_bb.y_max - in_bb.y_min) / NumberOfPointsPerAxis;
//    double z_increment = (in_bb.z_max - in_bb.z_min) / NumberOfPointsPerAxis;

//    for(double x = in_bb.x_min; x < in_bb.x_max; x+=x_increment){
//        for(double y = in_bb.y_min; y < in_bb.y_max; y+=y_increment){
//            for(double z = in_bb.z_min; z < in_bb.z_max; z+=z_increment){
//                pcl::PointXYZ pt(x,y,z);
//                pc.push_back(pt);
//            }
//        }
//    }

    pcl::PointXYZRGB pt;
    pt.x = in_bb.x_min; pt.y = in_bb.y_min; pt.z = in_bb.z_min; pc.push_back(pt);
    pt.x = in_bb.x_min; pt.y = in_bb.y_min; pt.z = in_bb.z_max; pc.push_back(pt);
    pt.x = in_bb.x_min; pt.y = in_bb.y_max; pt.z = in_bb.z_min; pc.push_back(pt);
    pt.x = in_bb.x_min; pt.y = in_bb.y_max; pt.z = in_bb.z_max; pc.push_back(pt);

    pt.x = in_bb.x_max; pt.y = in_bb.y_min; pt.z = in_bb.z_min; pc.push_back(pt);
    pt.x = in_bb.x_max; pt.y = in_bb.y_min; pt.z = in_bb.z_max; pc.push_back(pt);
    pt.x = in_bb.x_max; pt.y = in_bb.y_max; pt.z = in_bb.z_min; pc.push_back(pt);
    pt.x = in_bb.x_max; pt.y = in_bb.y_max; pt.z = in_bb.z_max; pc.push_back(pt);

    cout << "bbox published as pointcloud of size " << pc.size() << endl;
    bb_publisher.publish(pc);

}


int main(int argc, char** argv)
{


    ros::init(argc,argv,"moveit_jaco_listener");

    ros::NodeHandle nh;

    ros::Subscriber subA = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth_registered/points", 1, scene_callback);
    ros::Subscriber subB = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/grasp_object", 1, object_callback);

    pc_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/point_cloud_minus_object", 1);
    bb_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/object_to_grasp_bounding_box_DEBUG", 1);

//    myBoundingBox.x_min = 0;
//    myBoundingBox.x_max = 2;
//    myBoundingBox.y_min = 0;
//    myBoundingBox.y_max = 2;
//    myBoundingBox.z_min = 0;
//    myBoundingBox.z_max = 2;

//    ros::Rate r(5);
//    while(ros::ok()){
//        ros::spinOnce();
//        r.sleep();
//    }

    ros::spin();

    return 0;
}





















