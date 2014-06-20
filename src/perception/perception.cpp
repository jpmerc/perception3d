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

    ros::Subscriber sub4 = n.subscribe("/camera/rgb/image_color", 1, &ObjectExtractor::callback_rgb_camera, OBJ_EXTRACTOR_PTR);
    ros::Subscriber sub5 = n.subscribe("/coordinate_sender", 1, &ObjectExtractor::callback_coordinate_android, OBJ_EXTRACTOR_PTR);

    ros::Publisher pub_image = n.advertise<sensor_msgs::Image>("/square_image",1);

    ros::Publisher pub_android = n.advertise<std_msgs::String>("/android_listener",1);

   // boost::thread thread_(sendCommandsToJaco,JACO_PTR,2.3);
    JACO_PTR->close_fingers();


    // Spin threads
    ros::Rate r(5);
    while (ros::ok() && !OBJ_EXTRACTOR_PTR->pclViewer->wasStopped()) {
        ros::spinOnce();
        OBJ_EXTRACTOR_PTR->pclViewer->spinOnce (100);
        if(OBJ_EXTRACTOR_PTR->is_coordinate_received())
        {
            OBJ_EXTRACTOR_PTR->coordinate_processing();
            std_msgs::String response;
            response.data = "p_un;";
            pub_android.publish(response);
            response.data = "object_recon";
            pub_android.publish(response);
        }
        if(OBJ_EXTRACTOR_PTR->is_point_cloud_received())
        {
            OBJ_EXTRACTOR_PTR->point_cloud_processing();
            pub_image.publish(OBJ_EXTRACTOR_PTR->get_image_input());
        }
        else
        {
            pub_image.publish(OBJ_EXTRACTOR_PTR->get_image_memory());
        }
        OBJ_EXTRACTOR_PTR->set_point_cloud_received();
        OBJ_EXTRACTOR_PTR->set_coordinate_received();
        r.sleep();
    }

//    thread_.join();

    return 0;
}
