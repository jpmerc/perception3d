#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;
void printPose(string str, tf::Transform &in_pose);


void cameraToARTagTF(){
    tf::StampedTransform artag_kinect_referential;
    tf::TransformListener listener_cam_ar;
    static tf::TransformBroadcaster br;

    ros::Rate r(5);
    while(ros::ok()){
        bool arTagFound = listener_cam_ar.waitForTransform("camera_link","AR_OBJECT",ros::Time(0),ros::Duration(3.0));
        if(arTagFound){
            listener_cam_ar.lookupTransform("camera_link","AR_OBJECT",ros::Time(0),artag_kinect_referential);

            // Transforms AR_OBJECT to the same orientation of camera_link frame to facilitate the calculations afterwards
            artag_kinect_referential *= tf::Transform(tf::createQuaternionFromRPY(angles::from_degrees(-90),angles::from_degrees(90),angles::from_degrees(0)));

            // Publish tf to visualize the results in rviz

            br.sendTransform(tf::StampedTransform(artag_kinect_referential, ros::Time::now(), "camera_link", "AR_OBJECT_REORIENTED"));
        }
        r.sleep();
    }
}

void jacoToARTagTF(){
    tf::StampedTransform artag_jaco_referential;
    tf::TransformListener listener;
    tf::Transform artag_same_orientation;
    static tf::TransformBroadcaster br5;

    ros::Rate r(5);
    while(ros::ok()){
        bool arTagFound = listener.waitForTransform("arm_base","ARtag",ros::Time(0),ros::Duration(3.0));
        if(arTagFound){
            listener.lookupTransform("arm_base","ARtag",ros::Time(0),artag_jaco_referential);

            // ARtag with good orientation
            artag_same_orientation = tf::Transform(tf::createQuaternionFromRPY(angles::from_degrees(-90),angles::from_degrees(0),angles::from_degrees(-90)));
            artag_jaco_referential *= artag_same_orientation;

            br5.sendTransform(tf::StampedTransform(artag_jaco_referential, ros::Time::now(), "arm_base", "ARtag_REORIENTED"));
        }
    }
    r.sleep();
}



void alignARTagOfBothReferentials(){
    tf::TransformListener listener;
    tf::StampedTransform ar_kinect;
    tf::Transform diff;
    static tf::TransformBroadcaster br;
    ros::Rate r(10);

    while(ros::ok()){
        bool arTagFound = listener.waitForTransform("ARtag_REORIENTED","AR_OBJECT_REORIENTED",ros::Time(0),ros::Duration(3.0));
        if(arTagFound){
            listener.lookupTransform("ARtag_REORIENTED","AR_OBJECT_REORIENTED",ros::Time(0),ar_kinect);
            tf::Vector3 translation = ar_kinect.getOrigin();
            diff = tf::Transform(ar_kinect.getRotation(), translation);
            br.sendTransform(tf::StampedTransform(diff,ros::Time::now(),"ARtag_REORIENTED","ARtag_OFFSET"));
        }
        r.sleep();
    }
}

void printJacoKinectTF(){
    tf::TransformListener listener;
    tf::StampedTransform ar_kinect;
    tf::StampedTransform ar_jaco;
    tf::Transform add;
    static tf::TransformBroadcaster br;

    ros::Rate r(10);

    while(ros::ok()){
        bool arTagFound = listener.waitForTransform("AR_OBJECT_REORIENTED","camera_link",ros::Time(0),ros::Duration(3.0));
        if(arTagFound){
            listener.lookupTransform("AR_OBJECT_REORIENTED","camera_link",ros::Time(0),ar_kinect);

            listener.waitForTransform("arm_base","AR_OBJECT_REORIENTED",ros::Time(0),ros::Duration(3.0));
            listener.lookupTransform("arm_base","AR_OBJECT_REORIENTED",ros::Time(0),ar_jaco);

           // printPose("kinect",ar_kinect);
           // printPose("jaco",ar_jaco);

            //ar_kinect *= ar_jaco;

            tf::Vector3 translation = ar_kinect.getOrigin() + ar_jaco.getOrigin();
            add = tf::Transform(ar_kinect.getRotation()*ar_jaco.getRotation(), translation);
            br.sendTransform(tf::StampedTransform(add,ros::Time::now(),"arm_base","camera_link_calculated"));



            printPose("Calculated Camera Pose Relative to Arm_base", add);

        }
        r.sleep();
    }
}

void printPose(string str, tf::Transform &in_pose){
    cout << str << " : " << endl;
    cout << "x: " << in_pose.getOrigin().getX() << endl;
    cout << "y: " << in_pose.getOrigin().getY() << endl;
    cout << "z: " << in_pose.getOrigin().getZ() << endl;
    cout << "rotx: " << in_pose.getRotation().getX() << endl;
    cout << "roty: " << in_pose.getRotation().getY() << endl;
    cout << "rotz: " << in_pose.getRotation().getZ() << endl;
    cout << "rotw: " << in_pose.getRotation().getW() << endl;

    double roll,pitch,yaw;
    in_pose.getBasis().getRPY(roll,pitch,yaw);

    cout << "Yaw: "   << angles::to_degrees(yaw)   << " (" << yaw   << ")"   << endl;
    cout << "Pitch: " << angles::to_degrees(pitch) << " (" << pitch << ")"   << endl;
    cout << "Roll: "  << angles::to_degrees(roll)  << " (" << roll  << ")"   << endl;

    cout << endl;

}




int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "calibration");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    boost::thread kinect_tag(cameraToARTagTF);
    boost::thread jaco_tag(jacoToARTagTF);
    boost::thread align_tag(alignARTagOfBothReferentials);
    boost::thread findJacoKinectTF(printJacoKinectTF);


    ros::Rate r(5);
    while(ros::ok()){
        r.sleep();
    }


    return 0;
}
