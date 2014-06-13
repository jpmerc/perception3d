#include<ros/ros.h>

#include<image_transport/image_transport.h>

image_transport::Publisher pub;


void call_back_distance(const sensor_msgs::ImageConstPtr& p_input)
{
    pub.publish(p_input);
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/square_image",1,call_back_distance);
    pub = it.advertise("/rgb_image_square",1);
    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

