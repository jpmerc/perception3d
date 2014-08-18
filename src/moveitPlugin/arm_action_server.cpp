#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <jaco_msgs/JointVelocity.h>
#include <jaco_msgs/JointAngles.h>
#include <jaco_msgs/ArmJointAnglesAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <angles/angles.h>


ros::Publisher pub;
ros::Publisher pubTest;
float PI = 3.14159265358979323846;

/** \addtogroup container_ops_grp
  * @{ */

  /** Modifies the given angle to translate it into the [0,2pi[ range.
  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
  */
  template <class T>
  inline void wrapTo2PiInPlace(T &a)
  {
  bool was_neg = a<0;
  a = fmod(a, static_cast<T>(2.0*PI) );
  if (was_neg) a+=static_cast<T>(2.0*PI);
  }

  /** Modifies the given angle to translate it into the [0,2pi[ range.
  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
  */
  template <class T>
  inline T wrapTo2Pi(T a)
  {
  wrapTo2PiInPlace(a);
  return a;
  }

  /** Modifies the given angle to translate it into the ]-pi,pi] range.
  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
  * \sa wrapTo2Pi, wrapToPiInPlace, unwrap2PiSequence
  */
  template <class T>
  inline T wrapToPi(T a)
  {
  return wrapTo2Pi( a + static_cast<T>(PI) )-static_cast<T>(PI);
}


void callBackVelocity(const control_msgs::FollowJointTrajectoryGoalConstPtr& p_input, actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* p_server)
{
    std::cout << "Message received" << std::endl;
    std::vector<std::string> jointVector(p_input->trajectory.joint_names);

    std::vector<trajectory_msgs::JointTrajectoryPoint> pointVector(p_input->trajectory.points);

    //debug////////////////////////////////////////////////////////
    for(int i = 0; i < jointVector.size(); i++)
    {
        std::cout << jointVector[i] << std::endl;
    }

    std::cout << "JointTrajectorySize == " << pointVector.size() << std::endl;
    //////////////////////////////////////////////////////////////////


    ros::Duration memDuration(0);

    for(int i = 0; i < pointVector.size(); i++)
    {
        jaco_msgs::JointVelocity sendedMessage;
        std::vector<double> velocitiesVec(p_input->trajectory.points[i].velocities);
        //debug/////////////////////////////
        for(int j = 0; j < velocitiesVec.size(); j++)
        {
            std::cout << "velocities : " << velocitiesVec[j] << std::endl;
        }
        std::cout << "Duration : " << p_input->trajectory.points[i].time_from_start << std::endl;
        ////////////////////////////////////////////////////////////////////////////////////////////

        ros::Duration duration = p_input->trajectory.points[i].time_from_start;

        ros::Duration diff = (duration - memDuration);
        memDuration = p_input->trajectory.points[i].time_from_start;


        ros::Time timeToSend = ros::Time::now();
        timeToSend += diff;
        std::cout << "TimeToSend == " << timeToSend << std::endl;
        std::cout << "Diff ==" << diff << std::endl;


        sendedMessage.joint1 = (velocitiesVec[0] * 180)/PI;
        sendedMessage.joint2 = (velocitiesVec[1] * 180)/PI;
        sendedMessage.joint3 = (velocitiesVec[2] * 180)/PI;
        sendedMessage.joint4 = (velocitiesVec[3] * 180)/PI;
        sendedMessage.joint5 = (velocitiesVec[4] * 180)/PI;
        sendedMessage.joint6 = (velocitiesVec[5] * 180)/PI;
        while(ros::Time::now() < timeToSend)
        {
            pubTest.publish(sendedMessage);
            pub.publish(sendedMessage);
        }
    }

    //might send it earlier

    std::vector<double> position((p_input->trajectory.points[(pointVector.size())-1]).positions);

    for(int i = 0; i < position.size(); i++)
    {
        std::cout << "Position [" << i << "] == " << position[i] << std::endl;
    }

    control_msgs::FollowJointTrajectoryResult result;
    p_server->setSucceeded(result);

}


void callBackAngles(const control_msgs::FollowJointTrajectoryGoalConstPtr& p_input, actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* p_server)
{

    actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/jaco_arm_driver/joint_angles/arm_joint_angles", true);

    std::vector<trajectory_msgs::JointTrajectoryPoint> pointVector(p_input->trajectory.points);

    for(int i = 0; i < pointVector.size(); i++)
    {
        std::vector<double> position(p_input->trajectory.points[i].positions);

        jaco_msgs::ArmJointAnglesGoal sendPosition;

        sendPosition.angles.joint1 = position[0];
        sendPosition.angles.joint2 = position[1];
        sendPosition.angles.joint3 = position[2];
        sendPosition.angles.joint4 = position[3];
        sendPosition.angles.joint5 = position[4];
        sendPosition.angles.joint6 = position[5];

        ac.sendGoal(sendPosition);
        //debug

        std::cout << "Position1 == " << position[0] << std::endl;
        std::cout << "Position2 == " << position[1] << std::endl;
        std::cout << "Position3 == " << position[2] << std::endl;
        std::cout << "Position4 == " << position[3] << std::endl;
        std::cout << "Position5 == " << position[4] << std::endl;
        std::cout << "Position6 == " << position[5] << std::endl;
        std::cout << "===============================" << std::endl;
        /////////////////////////////////////////////////////////////

        ac.waitForResult(ros::Duration(1.0));

    }
    control_msgs::FollowJointTrajectoryResult result;
    p_server->setSucceeded(result);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_controller");

    ros::NodeHandle nh;

    pub = nh.advertise<jaco_msgs::JointVelocity>("jaco_arm_driver/in/joint_velocity",1);
    pubTest = nh.advertise<jaco_msgs::JointVelocity>("/jaco_arm_driver/in/joint_velocity_test",1);

    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>as(nh, "arm/arm_joint_angles",boost::bind(&callBackAngles,_1, &as),false);

    as.start();

    while(ros::ok())
    {
        ros::spin();
    }


    return 0;
}
