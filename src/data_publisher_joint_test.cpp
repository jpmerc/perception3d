#include <ros/ros.h>
#include <jaco_msgs/JointVelocity.h>

/*
  This is a script to test the joint velocity.  You can choose how many velocity you want to send.
  It will send it for 2 sec.  The script will first send 0 and will finish with 0.  You can send only
  to 1 joint during the entire process.

  To launch it, you will need to use the launch file data_publisher_joint_test.launch.
  In there you can specify the joint you want to move.
  Param = joint_name.  range value = [1,6].

  The velocity is specify with the other param.
  Param = joint_velocity_number#  # = [0,1,2,3,4,...]  The value represent the velocity you want
  to set the joint to (deg/s).  The 0 deg/s at the begining and at the end is by default set.
  */


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_publisher_joint_test");

    ros::NodeHandle nh;
    ros::NodeHandle nParam("~");
    ros::Publisher pub = nh.advertise<jaco_msgs::JointVelocity>("/jaco_arm_driver/in/joint_velocity",1);
    ros::Rate r(1000);

    int jointName;
    nParam.param("joint_name", jointName, 1);

    int jointVelocityNumber;
    nParam.param("joint_velocity_number", jointVelocityNumber, 1);

    std::vector<jaco_msgs::JointVelocity> jointVelocityVec;

    for(int i = 0; i < jointVelocityNumber; i++)
    {
        std::stringstream ss;
        ss << "joint_velocity" << i;
        int jointVelocity;
        nParam.param(ss.str().c_str(), jointVelocity, 0);

        std::cout << "Joint velocity  :  " << i << ss.str() << std::endl;
        std::cout << "Velocity :  " << jointVelocity << std::endl;

        jaco_msgs::JointVelocity sendMessage;
        if(jointName == 1)
        {
            sendMessage.joint1 =jointVelocity;
            sendMessage.joint2 =0;
            sendMessage.joint3 =0;
            sendMessage.joint4 =0;
            sendMessage.joint5 =0;
            sendMessage.joint6 =0;
        }
        else if(jointName == 2)
        {
            sendMessage.joint1 =0;
            sendMessage.joint2 =jointVelocity;
            sendMessage.joint3 =0;
            sendMessage.joint4 =0;
            sendMessage.joint5 =0;
            sendMessage.joint6 =0;
        }
        else if(jointName == 3)
        {
            sendMessage.joint1 =0;
            sendMessage.joint2 =0;
            sendMessage.joint3 =jointVelocity;
            sendMessage.joint4 =0;
            sendMessage.joint5 =0;
            sendMessage.joint6 =0;
        }
        else if(jointName == 4)
        {
            sendMessage.joint1 =0;
            sendMessage.joint2 =0;
            sendMessage.joint3 =0;
            sendMessage.joint4 =jointVelocity;
            sendMessage.joint5 =0;
            sendMessage.joint6 =0;
        }
       else if(jointName == 5)
        {
            sendMessage.joint1 =0;
            sendMessage.joint2 =0;
            sendMessage.joint3 =0;
            sendMessage.joint4 =0;
            sendMessage.joint5 =jointVelocity;
            sendMessage.joint6 =0;
        }
        else if(jointName == 6)
        {
            sendMessage.joint1 =0;
            sendMessage.joint2 =0;
            sendMessage.joint3 =0;
            sendMessage.joint4 =0;
            sendMessage.joint5 =0;
            sendMessage.joint6 =jointVelocity;
        }

        jointVelocityVec.push_back(sendMessage);
    }

    jaco_msgs::JointVelocity sendMessageZero;
    sendMessageZero.joint1 = 0;
    sendMessageZero.joint2 = 0;
    sendMessageZero.joint3 = 0;
    sendMessageZero.joint4 = 0;
    sendMessageZero.joint5 = 0;
    sendMessageZero.joint6 = 0;

    ros::Time duration2 = ros::Time::now() + ros::Duration(2,0);
    while(ros::Time::now() < duration2)
    {
        pub.publish(sendMessageZero);
        r.sleep();
    }

    for(int i = 0; i < jointVelocityVec.size(); i++)
    {
        ros::Time duration22 = ros::Time::now() + ros::Duration(2,0);
        while(ros::Time::now() < duration22)
        {
            pub.publish(jointVelocityVec[i]);
            r.sleep();
        }
    }

    ros::Time duration222 = ros::Time::now() + ros::Duration(2,0);
    while(ros::Time::now() < duration222)
    {
        pub.publish(sendMessageZero);
        r.sleep();
    }

    return 0;
}
