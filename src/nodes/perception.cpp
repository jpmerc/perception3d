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
    nh.param("objects_visualizer",show_objects_in_viewer,false);




    obj_extractor = new ObjectExtractor(show_objects_in_viewer);
    ros::Subscriber sub = n.subscribe ("/custom/not_planes", 1, &ObjectExtractor::extraction_callback, obj_extractor);




    cout << *obj_extractor->get_showUI() << endl;

    // Spin threads
    //ros::spin();

    return 0;
}
