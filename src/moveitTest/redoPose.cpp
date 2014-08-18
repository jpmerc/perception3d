#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<jaco_msgs/ArmJointAnglesAction.h>
#include<control_msgs/FollowJointTrajectoryAction.h>


#include <fstream>

/*
  This is a script to redo a movement from a text file.  It will load a text file and send te command
  to the arm.  To launch it, set the log_file in the launch file.  Set the exact path to the text
  file to load.
  */


void parseTextFile(std::string p_path, std::vector<jaco_msgs::ArmJointAnglesGoal>& p_vectorGoal)
{
    std::ifstream ifs;
    ifs.open(p_path.c_str());
    if(ifs.is_open())
    {
        std::string line;
        int nbLine = 0;
        jaco_msgs::ArmJointAnglesGoal goalTemp;
        std::vector<double> position;
        while(std::getline(ifs, line))
        {
            if(nbLine == 6)
            {
                nbLine = 0;
                goalTemp.angles.joint1 = position[0];
                goalTemp.angles.joint2 = position[1];
                goalTemp.angles.joint3 = position[2];
                goalTemp.angles.joint4 = position[3];
                goalTemp.angles.joint5 = position[4];
                goalTemp.angles.joint6 = position[5];
                position.clear();
                p_vectorGoal.push_back(goalTemp);
            }
            else
            {
                position.push_back(atof(line.c_str()));
                nbLine++;
            }
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "redo_pose_action");

    ros::NodeHandle nParam("~");
    ros::NodeHandle nh;

    std::string logFile;
    nParam.param("log_file", logFile, std::string(""));

    actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/jaco_arm_driver/joint_angles/arm_joint_angles", true);


    std::vector<jaco_msgs::ArmJointAnglesGoal> pointVector;
    parseTextFile(logFile, pointVector);
    std::cout << "File : " << logFile << std::endl;

    std::cout << "Point size : " << pointVector.size() << std::endl;

    for(int i = 0; i < pointVector.size(); i++)
    {
        ac.sendGoal(pointVector.at(i));
        //debug
        std::cout << ros::Time::now() << std::endl;
        std::cout << "Position1 == " << pointVector.at(i).angles.joint1 << std::endl;
        std::cout << "Position2 == " << pointVector.at(i).angles.joint2 << std::endl;
        std::cout << "Position3 == " << pointVector.at(i).angles.joint3 << std::endl;
        std::cout << "Position4 == " << pointVector.at(i).angles.joint4 << std::endl;
        std::cout << "Position5 == " << pointVector.at(i).angles.joint5 << std::endl;
        std::cout << "Position6 == " << pointVector.at(i).angles.joint6 << std::endl;
        std::cout << "===============================" << std::endl;
        /////////////////////////////////////////////////////////////

        ac.waitForResult(ros::Duration(2.0));
    }

    return 0;
}
