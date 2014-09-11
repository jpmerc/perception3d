#include <ros/ros.h>
#include <objectExtractor.h>
#include <communication.h>
#include <jaco_custom.h>
#include <fileAPI.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

ObjectExtractor *OBJ_EXTRACTOR_PTR;
JacoCustom *JACO_PTR;
FileAPI *API_PTR;
ros::CallbackQueue jaco_callbacks;

using namespace std;

void callbackThread(){
    ros::NodeHandle n;ros::Rate r(5);
    while(n.ok()){
        jaco_callbacks.callAvailable(ros::WallDuration(0));
        r.sleep();
    }
}

void trainFunctionTestThread(Communication *communication_ptr){
    ros::NodeHandle n;ros::Rate r(10);sleep(10);
//    while(n.ok()){
//        //communication_ptr->testTFandSurfaceTransforms();
//        communication_ptr->train(true,true);
//        r.sleep();
//    }


//    communication_ptr->train(true,true);



//    int count = 0;
//    int timeToWait = 60;
//    while(true){
//        sleep(1);
//        cout << count++ << endl;
//        if(count >= timeToWait) break;
//    }

//    sleep(10);
    communication_ptr->repeat();
    cout << "The arm finished moving" << endl;
}

void recognitionTestsThread(Communication *communication_ptr){
    ros::NodeHandle n;
    ros::Rate r(5);
    int count = 0;
    int timeToWait = 60;
    while(true){
        sleep(1);
        cout << count++ << endl;
        if(count >= timeToWait) break;
    }
    while(n.ok()){
            communication_ptr->testRecognition();
           // communication_ptr->recognitionViewer->spinOnce();
            r.sleep();
        }
}

void sendCommandsToJaco(JacoCustom *jaco){
    while(ros::ok()){
        std::string input;
        std::cout << "Enter your command : " << std::endl;
        std::cin >> input;
        if(input == "q" || input == "exit" || input == "quit"){
            break;
        }
        else{
            std::string axis    = input.substr(0,1);
            std::string distance = input.substr(1,input.size()-1);
            if(axis == "x" || axis == "y" || axis == "z"){
                double dist = atof(distance.c_str());
                jaco->moveAlongAxis(axis,dist);
            }
        }
    }
}


int main (int argc, char** argv){
    ros::init (argc, argv, "perception");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Load parameters from launch file
    bool show_objects_in_viewer;
    string directory_url;
    nh.param("objects_visualizer",show_objects_in_viewer,true);
    nh.param("directory_url", directory_url, string("/home/jp/devel/src/perception3d/database"));

    // Class Instances
    JACO_PTR = new JacoCustom(n);
    OBJ_EXTRACTOR_PTR = new ObjectExtractor(show_objects_in_viewer, n);
    API_PTR = new FileAPI(directory_url);
    //API_PTR = new FileAPI();
    Communication* communication_ptr = new Communication(OBJ_EXTRACTOR_PTR, API_PTR, JACO_PTR);

    // Ros Subscribers (global callbacks)
    ros::Subscriber sub1 = n.subscribe("/custom/not_planes", 1, &ObjectExtractor::extraction_callback, OBJ_EXTRACTOR_PTR);
    ros::Subscriber sub2 = n.subscribe("/camera/rgb/image_color", 1, &ObjectExtractor::callback_rgb_camera, OBJ_EXTRACTOR_PTR);
    ros::Subscriber sub3 = n.subscribe("/android_sender", 1, &Communication::callback_android_listener, communication_ptr);

    // Different Callback Queue for Jaco Callbacks (position)
    const std::string arm_topic = "/jaco_arm_driver/out/tool_position";
    const std::string fingers_topic = "/jaco_arm_driver/out/finger_position";
    ros::SubscribeOptions fingers = ros::SubscribeOptions::create<jaco_msgs::FingerPosition>(fingers_topic,1,boost::bind(&JacoCustom::fingers_position_callback,JACO_PTR,_1),ros::VoidPtr(),&jaco_callbacks);
    ros::Subscriber sub_f = n.subscribe(fingers);
    ros::SubscribeOptions arm = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(arm_topic,1,boost::bind(&JacoCustom::arm_position_callback,JACO_PTR,_1),ros::VoidPtr(),&jaco_callbacks);
    ros::Subscriber sub_a = n.subscribe(arm);
    boost::thread spin_thread(callbackThread);

    //Thread to test the training phase of the system
   boost::thread trainTest(trainFunctionTestThread,communication_ptr);
    //boost::thread recognitionTest(recognitionTestsThread,communication_ptr);
  //  boost::thread(sendCommandsToJaco,JACO_PTR);

    // Spin threads
    ros::Rate r(5);
    while (ros::ok() && !OBJ_EXTRACTOR_PTR->pclViewer->wasStopped()) {
        ros::spinOnce();
        OBJ_EXTRACTOR_PTR->pclViewer->spinOnce (100);
        //communication_ptr->recognitionViewer->spinOnce(100);
        communication_ptr->spin_once();
        r.sleep();
    }

    return 0;
}



