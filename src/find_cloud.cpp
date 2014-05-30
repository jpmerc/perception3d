#include<ros/ros.h>

#include<sensor_msgs/Image.h>
#include<std_msgs/UInt32MultiArray.h>

#include<cstdlib>

ros::Publisher PUB;
ros::Publisher PUB2;

void callback_function(const sensor_msgs::Image& p_input)
{
    //**********///
    //would need error traitement for the input
    //***********////


    int coordinate[2];
    std::string input;
    std::cin >> input;
    std::string buffer = "";
    int indice = 0;
    for(int i = 0; i < input.length(); i ++)
    {
        if(input.at(i) != '.')
            buffer += input[i];
        else
        {
            coordinate[indice] = atoi(buffer.c_str());
            buffer.clear();
            indice ++;
        }
    }
    coordinate[1] = atoi(buffer.c_str());

    std_msgs::UInt32MultiArray array_send;
    array_send.data.push_back(coordinate[0]);
    array_send.data.push_back(coordinate[1]);

    std::cout << array_send.data.at(0) << std::endl;
    std::cout << array_send.data.at(1) << std::endl;
    PUB.publish(array_send);
}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "point_cloud_finder");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/square_image", 1, callback_function);

    PUB = nh.advertise<std_msgs::UInt32MultiArray> ("/image_coordinate_rgb", 1);


    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
