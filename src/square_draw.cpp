#include<ros/ros.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>
#include<sensor_msgs/Image.h>


typedef pcl::PointXYZRGB PointT;

pcl::PointCloud<PointT>::Ptr point_cloud_input(new pcl::PointCloud<PointT>);

pcl::PointCloud<PointT>::Ptr point_cloud_diffuse_test(new pcl::PointCloud<PointT>);

ros::Publisher pub;
ros::Publisher pub2;

void draw_square(const PointT& p_left, const PointT& p_right, const PointT& p_top, const PointT& p_bottom)
{
    PointT top_right(255,0,0);
    top_right.x = p_left.x;
    top_right.y = p_top.y;
    top_right.z = 0;

    PointT top_left(0,255,0);
    top_left.x = p_right.x;
    top_left.y = p_top.y;
    top_left.z = 0;

    PointT bottom_right(0,0,255);
    bottom_right.x = p_left.x;
    bottom_right.y = p_bottom.y;
    bottom_right.z = 0;

    PointT bottom_left(255,255,0);
    bottom_left.x = p_right.x;
    bottom_left.y = p_bottom.y;
    bottom_left.z = 0;

    point_cloud_diffuse_test->points.push_back(top_left);
    point_cloud_diffuse_test->points.push_back(top_right);
    point_cloud_diffuse_test->points.push_back(bottom_left);
    point_cloud_diffuse_test->points.push_back(bottom_right);
    //std::cout << p_bottom.y << std::endl;// debug
}

void callback_function(const pcl::PCLPointCloud2ConstPtr& p_input)
{
    point_cloud_diffuse_test->clear();

    pcl::fromPCLPointCloud2(*p_input, *point_cloud_input);
    PointT left;
    PointT right;
    PointT top;
    PointT bottom;
    int compteur = 0;
    for(int i = 0; i < point_cloud_input->points.size(); i++)
    {
        switch(compteur)
        {
        case 0: left = point_cloud_input->points.at(i); break;
        case 1: right = point_cloud_input->points.at(i); break;
        case 2: top = point_cloud_input->points.at(i); break;
        case 3: bottom = point_cloud_input->points.at(i); break;
        }

        compteur++;
        if(compteur == 4)
        {
            draw_square(left, right, top, bottom);
            compteur = 0;
        }
    }
    pcl::PCLPointCloud2 pointCloud2;
    pcl::toPCLPointCloud2(*point_cloud_diffuse_test, pointCloud2);
    sensor_msgs::PointCloud2 send;
    pcl_conversions::fromPCL(pointCloud2, send);
    pcl_conversions::fromPCL(p_input->header,send.header);
    send.header.frame_id = "/camera_rgb_optical_frame";
    pub.publish(send);

}

void callback_function2(const sensor_msgs::ImageConstPtr& p_input)
{
    sensor_msgs::Image im;
    im = *p_input;
    for(int i = 0; i < 921600; i ++)
    {
        if(i % 3 == 0)
        {
        im.data.at(i) = 0;
        }
    }
    pub2.publish(im);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_drawing");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/corner_limits", 1, callback_function);

    ros::Subscriber sub2 = nh.subscribe("/camera/rgb/image_color", 1, callback_function2);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/square_test", 1);

    pub2 = nh.advertise<sensor_msgs::Image> ("/image_test", 1);

    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
