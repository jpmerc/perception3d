#include <ros/ros.h>
#include <objectExtractor.h>
#include <jaco_custom.h>


ObjectExtractor *OBJ_EXTRACTOR_PTR;
JacoCustom *JACO_PTR;

//void sendCommandsToJaco(JacoCustom* jaco,double x){
//    if(ros::ok()){
//         jaco->open_fingers();
////        move_to_grasp_point(-0.24, 0.366, -0.003, 0.064, -0.658, -0.035, 0.75);
//        //jaco->close_fingers();
////        move_up(0.4);
////        end_program = true;
//    }
//}



using namespace std;

int main (int argc, char** argv){
    ros::init (argc, argv, "perception");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");



    // Load parameters from launch file
    bool show_objects_in_viewer;
    nh.param("objects_visualizer",show_objects_in_viewer,true);


    OBJ_EXTRACTOR_PTR = new ObjectExtractor(show_objects_in_viewer);
    ros::Subscriber sub = n.subscribe("/custom/not_planes", 1, &ObjectExtractor::extraction_callback, OBJ_EXTRACTOR_PTR);

    JACO_PTR = new JacoCustom();
    ros::Subscriber sub2 = n.subscribe("/jaco/tool_position", 1, &JacoCustom::arm_position_callback, JACO_PTR);
    ros::Subscriber sub3 = n.subscribe("/jaco/finger_position", 1, &JacoCustom::fingers_position_callback, JACO_PTR);

   // boost::thread thread_(sendCommandsToJaco,JACO_PTR,2.3);
    JACO_PTR->close_fingers();


    // Spin threads
    ros::Rate r(5);
    while (ros::ok() && !OBJ_EXTRACTOR_PTR->pclViewer->wasStopped()) {
        ros::spinOnce();
        OBJ_EXTRACTOR_PTR->pclViewer->spinOnce (100);
        r.sleep();
    }

//    thread_.join();

    return 0;
}
