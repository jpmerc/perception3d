#include <ros/ros.h>
#include <planSegmentor.h>

int main(int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "segmentation");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    planSegmentor* planExtractor_ptr = new planSegmentor(n);

    bool testValidation;
    nh.param("test", testValidation, false);

    bool showUI;
    // Load parameters from launch file
    nh.param("pcl_visualizer",showUI,true);

    planExtractor_ptr->setShowUi(showUI);
    planExtractor_ptr->setPCLViewer();

    bool loop_condition = true;
    ros::Rate r(30);

    if(!testValidation)
    {
        std::string topic_in;

        // Create a ROS subscriber for the input point cloud
        nh.param("topic_in",topic_in,std::string("/camera/depth_registered/points"));
        ros::Subscriber sub = n.subscribe (topic_in,
                                           1,
                                           &planSegmentor::cloud_callback,
                                           planExtractor_ptr);

        while (loop_condition)
        {
            ros::spinOnce();
            if(showUI)
            {
                planExtractor_ptr->viewerSpinOnce();
                loop_condition = ros::ok() && !planExtractor_ptr->viewerStoped();
            }
            else
            {
                loop_condition = ros::ok();
            }
            r.sleep();
        }
    }
    else
    {
        std::string path;
        nh.param("file_path", path, std::string(""));
        planExtractor_ptr->loadFile(path);

        while (loop_condition)
        {
            ros::spinOnce();
            planExtractor_ptr->spinOnceTestFile();
            r.sleep();
            loop_condition = ros::ok();
        }
    }




    return 0;
}
