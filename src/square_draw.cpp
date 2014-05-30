#include<ros/ros.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/range_image_visualizer.h>
#include<pcl/point_cloud.h>
#include<pcl/range_image/range_image.h>
#include<pcl/range_image/range_image_planar.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/UInt32MultiArray.h>

#include<boost/thread/mutex.hpp>


typedef pcl::PointXYZRGB PointT;

pcl::PointCloud<PointT>::Ptr POINT_CLOUD_INPUT(new pcl::PointCloud<PointT>);

pcl::PointCloud<PointT>::Ptr POINT_CLOUD_CORNER(new pcl::PointCloud<PointT>);

bool COORDINATE_RECEIVED = false;
std_msgs::UInt32MultiArray COORDINATE_USER_SENDED;


ros::Publisher PUB;
ros::Publisher PUB2;

boost::mutex mtx;

//find the square corner for an object
void find_corner(const PointT& p_left, const PointT& p_right, const PointT& p_top, const PointT& p_bottom)
{
    PointT top_right(255,0,0);
    top_right.x = p_left.x;
    top_right.y = p_top.y;
    top_right.z = 1;

    PointT top_left(0,255,0);
    top_left.x = p_right.x;
    top_left.y = p_top.y;
    top_left.z = 1;

    PointT bottom_right(0,0,255);
    bottom_right.x = p_left.x;
    bottom_right.y = p_bottom.y;
    bottom_right.z = 1;

    PointT bottom_left(255,255,0);
    bottom_left.x = p_right.x;
    bottom_left.y = p_bottom.y;
    bottom_left.z = 1;

    POINT_CLOUD_CORNER->points.push_back(top_left);
    POINT_CLOUD_CORNER->points.push_back(top_right);
    POINT_CLOUD_CORNER->points.push_back(bottom_left);
    POINT_CLOUD_CORNER->points.push_back(bottom_right);
}

//modifie the pixel at the given coordinate
void change_pixel_color(std::vector<unsigned char>& p_array, int p_x, int p_y, int p_b = 255, int p_g = 0, int p_r = 0)
{
    if(p_x < 0)
        p_x = 0;
    else if(p_x > 640)
        p_x = 640;
    if(p_y < 0)
        p_y = 0;
    if(p_y > 480)
        p_y = 480;


    int row = 3*p_x + 1920*p_y;
    p_array[row] = p_b;
    p_array[row + 1] = p_g;
    p_array[row + 2] = p_r;
}


//draw the square into the rgb image
void draw_square(std::vector<unsigned char>& p_array, PointT p_top_left, PointT p_top_right, PointT p_bottom_left, PointT p_bottom_right)
{
    //draw the upper line
    for(int i = p_top_left.x; i <= p_top_right.x; i++)
    {
        change_pixel_color(p_array, i, p_top_left.y);
    }
    //draw the down line
    for(int i = p_bottom_left.x; i <= p_bottom_right.x; i++)
    {
        change_pixel_color(p_array, i, p_bottom_left.y,0,255,0);
    }
    //draw the left line
    for(int i = p_top_left.y; i <= p_bottom_left.y; i++)
    {
        change_pixel_color(p_array, p_top_left.x, i,0,0,255);
    }
    //draw the right line
    for(int i = p_top_right.y; i <=p_bottom_right.y; i++)
    {
        change_pixel_color(p_array, p_top_right.x, i,255,255,0);
    }
}

//do nothing for now but will probaly publish a point cloud to recognise the object
void object_recognition()
{

}




void projection2d_pointCloud(const pcl::PointCloud<PointT>& p_input, pcl::PointCloud<PointT>& p_output)
{
    p_output.clear();
    for(int i = 0; i < p_input.size(); i++)
    {
        PointT point_3d = p_input.at(i);
        PointT point(1,1,1);
        point.x = point_3d.x / point_3d.z;
        point.y = point_3d.y / point_3d.z;
        point.z = point_3d.z / point_3d.z;
        p_output.push_back(point);
    }
}

/*
  input: point cloud with the object limits
  process: find the objetcs corners
  publish: point cloud with the all the corner in order top left, top right, bottom left, bottom right
*/
void callback_function(const pcl::PCLPointCloud2ConstPtr& p_input)
{
    ros::Duration d = ros::Duration(0,5);
    d.sleep();

    boost::unique_lock<boost::mutex> scoped_lock(mtx);
    POINT_CLOUD_CORNER->clear();
    pcl::fromPCLPointCloud2(*p_input, *POINT_CLOUD_INPUT);
    //find the corner on the 2d plan////////
    PointT left;
    PointT right;
    PointT top;
    PointT bottom;
    int compteur = 0;
    for(int i = 0; i < POINT_CLOUD_INPUT->points.size(); i++)
    {
        switch(compteur)
        {
        case 0: left = POINT_CLOUD_INPUT->points.at(i); break;
        case 1: right = POINT_CLOUD_INPUT->points.at(i); break;
        case 2: top = POINT_CLOUD_INPUT->points.at(i); break;
        case 3: bottom = POINT_CLOUD_INPUT->points.at(i); break;
        }

        compteur++;
        if(compteur == 4)
        {
            find_corner(left, right, top, bottom);
            compteur = 0;
        }
    }
    //////

    pcl::PCLPointCloud2 pointCloud2;
    pcl::toPCLPointCloud2(*POINT_CLOUD_CORNER, pointCloud2);
    sensor_msgs::PointCloud2 send;
    pcl_conversions::fromPCL(pointCloud2, send);
    pcl_conversions::fromPCL(p_input->header,send.header);
    send.header.frame_id = "/camera_rgb_optical_frame";
    PUB.publish(send);
}

/*
  input: rgb image from the kinect
  process: draw the square around the objects
  publish: rgb image with the square
*/

void callback_function2(const sensor_msgs::Image& p_input)
{
    ros::Duration d = ros::Duration(0,5);
    d.sleep();
    if(COORDINATE_RECEIVED)
    {
        object_recognition();
    }

    boost::unique_lock<boost::mutex> scoped_lock(mtx);
    sensor_msgs::Image im = p_input;

    int compteur = 0;
    PointT top_left(255,0,0);
    PointT top_right(255,0,0);
    PointT bottom_left(255,0,0);
    PointT bottom_right(255,0,0);
    for(int i = 0; i < POINT_CLOUD_CORNER->size(); i++)
    {
        switch(compteur)
        {
        case(0):top_left = POINT_CLOUD_CORNER->at(i); compteur++; break;
        case(1):top_right = POINT_CLOUD_CORNER->at(i);compteur++;break;
        case(2):bottom_left = POINT_CLOUD_CORNER->at(i);compteur++;break;
        case(3):bottom_right = POINT_CLOUD_CORNER->at(i);
            draw_square(im.data, top_left, top_right, bottom_left, bottom_right);
            compteur = 0;
            break;
        }
    }
    im.header = p_input.header;
    im.encoding = p_input.encoding;
    im.width = 640;
    im.height = 480;
    PUB2.publish(im);
}

void callback_function3(const std_msgs::UInt32MultiArray& p_input)
{

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_drawing");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/corner_limits", 1, callback_function);
    ros::Subscriber sub2 = nh.subscribe("/camera/rgb/image_color", 1, callback_function2);
    ros::Subscriber sub3 = nh.subscribe("/image_coordinate_rgb", 1, callback_function3);

    PUB = nh.advertise<sensor_msgs::PointCloud2> ("/corner_limits", 1);
    PUB2 = nh.advertise<sensor_msgs::Image> ("/square_image", 1);

    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
