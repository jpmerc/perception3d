#include <ros/ros.h>
#include <objectExtractor.h>


ObjectExtractor *obj_extractor;


using namespace std;

int main (int argc, char** argv){
    ros::init (argc, argv, "perception");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");



    // Load parameters from launch file
    bool show_objects_in_viewer;
    nh.param("objects_visualizer",show_objects_in_viewer,true);


    obj_extractor = new ObjectExtractor(show_objects_in_viewer);
    ros::Subscriber sub = n.subscribe ("/custom/not_planes", 1, &ObjectExtractor::extraction_callback, obj_extractor);


    // Spin threads
    ros::Rate r(5);

    while (ros::ok() && !obj_extractor->pclViewer->wasStopped()) {
        ros::spinOnce();
        obj_extractor->pclViewer->spinOnce (100);
        r.sleep();
    }

    return 0;
}
