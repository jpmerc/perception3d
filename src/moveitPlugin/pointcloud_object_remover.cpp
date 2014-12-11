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
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

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
ros::Publisher planning_scene_publisher;
struct BoundingBox myBoundingBox;
struct BoundingBox oldBoundinxBox;
int bbox_seq_number = 0;
double object_removal_padding_cm; // ROS PARAM
ros::ServiceClient client_get_scene_;

void resetOctomap();
void printBoundingBox(BoundingBox in_bb);
void publishBoundingBoxAsPointCloud(BoundingBox in_bb);
void addPaddingToBoundingBox(BoundingBox &in_bb);
bool areSameBoundingBoxes(BoundingBox bb1, BoundingBox bb2);



void object_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc){

    struct BoundingBox temp_BB = findBoundingBox(pc);

    object_mutex.lock();
    myBoundingBox = temp_BB;
    object_mutex.unlock();
}


void scene_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& scene_cloud){

    pcl::PointCloud<pcl::PointXYZRGB> pc = *scene_cloud;
    //cout << "Frame_id = " << pc.header.frame_id << endl;

    object_mutex.lock();
    struct BoundingBox BB = myBoundingBox;
    object_mutex.unlock();

    bool areTheSame = areSameBoundingBoxes(BB, oldBoundinxBox);
    oldBoundinxBox = BB;

    if(!areTheSame){
        resetOctomap();
    }

    addPaddingToBoundingBox(BB);

   // publishBoundingBoxAsPointCloud(BB);
   // printBoundingBox(BB);

    for(int i = 0; i < scene_cloud->points.size(); i++){

        double x = pc.points.at(i).x;
        double y = pc.points.at(i).y;
        double z = pc.points.at(i).z;

        if(x >= BB.x_min && x <= BB.x_max && y >= BB.y_min && y <= BB.y_max && z >= BB.z_min && z <= BB.z_max ){
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

void addPaddingToBoundingBox(BoundingBox &in_bb){

    double padding_m = object_removal_padding_cm / 100;

    in_bb.x_min -= padding_m;
    in_bb.x_max += padding_m;
    in_bb.y_min -= padding_m;
    in_bb.y_max += padding_m;
    in_bb.z_min -= padding_m;
    in_bb.z_max += padding_m;
}


/* Publishes an empty scene to reset/refresh it */
void resetOctomap(){
    cout << "Clearing Octomap !" << endl;
    moveit_msgs::PlanningScene planning_scene;

    moveit_msgs::GetPlanningScene scene_srv;
    scene_srv.request.components.components =   scene_srv.request.components.ALLOWED_COLLISION_MATRIX +
                                                scene_srv.request.components.LINK_PADDING_AND_SCALING +
                                                scene_srv.request.components.OBJECT_COLORS +
                                                scene_srv.request.components.OCTOMAP +
                                                scene_srv.request.components.ROBOT_STATE +
                                                scene_srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS +
                                                scene_srv.request.components.SCENE_SETTINGS +
                                                scene_srv.request.components.TRANSFORMS +
                                                scene_srv.request.components.WORLD_OBJECT_GEOMETRY +
                                                scene_srv.request.components.WORLD_OBJECT_NAMES;



    if(client_get_scene_.call(scene_srv)){
         moveit_msgs::PlanningScene planning_scene = scene_srv.response.scene;
         planning_scene.world.octomap.octomap.data.clear();
         ros::WallDuration sleep_time(0.5);
         //planning_scene.is_diff = true;
         planning_scene_publisher.publish(planning_scene);
         sleep_time.sleep();
    }
}


bool areSameBoundingBoxes(BoundingBox bb1, BoundingBox bb2){

    if(bb1.x_min == bb2.x_min &&
            bb1.y_min == bb2.y_min &&
            bb1.z_min == bb2.z_min &&
            bb1.x_max == bb2.x_max &&
            bb1.y_max == bb2.y_max &&
            bb1.z_max == bb2.z_max
            )
    {
        return true;
    }

    else {
        return false;
    }

}


int main(int argc, char** argv)
{


    ros::init(argc,argv,"point_cloud_object_remover");

    ros::NodeHandle nh;

    oldBoundinxBox.x_min = 0;
    oldBoundinxBox.x_max = 0;
    oldBoundinxBox.y_min = 0;
    oldBoundinxBox.y_max = 0;
    oldBoundinxBox.z_min = 0;
    oldBoundinxBox.z_max = 0;


    ros::Subscriber subA = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth_registered/points", 1, scene_callback);
    ros::Subscriber subB = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/grasp_object", 1, object_callback);

    pc_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/point_cloud_minus_object", 1);
    bb_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/object_to_grasp_bounding_box_DEBUG", 1);
    planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    client_get_scene_ = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

    nh.param<double>("object_removal_padding_cm", object_removal_padding_cm, 0.5);


//    myBoundingBox.x_min = 0;
//    myBoundingBox.x_max = 2;
//    myBoundingBox.y_min = 0;
//    myBoundingBox.y_max = 2;
//    myBoundingBox.z_min = 0;
//    myBoundingBox.z_max = 2;



    ros::Rate r(5);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

  //  ros::spin();

    return 0;
}





















