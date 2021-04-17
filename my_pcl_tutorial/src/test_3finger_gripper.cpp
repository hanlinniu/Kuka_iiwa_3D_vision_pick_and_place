#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <robotiq_s_model_control/SModel_robot_output.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher command_pub = n.advertise<robotiq_s_model_control::SModel_robot_output>("SModelRobotOutput", 1);

  ros::Rate loop_rate(10);


  robotiq_s_model_control::SModel_robot_output command_msg;



  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());



    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    chatter_pub.publish(msg);
    ros::Duration(1).sleep();


    // activate first
    command_msg.rACT = 1;
    command_msg.rMOD = 0;
    command_msg.rGTO = 1;
    command_msg.rATR = 0;
    command_msg.rPRA = 0;
    command_msg.rSPA = 255;
    command_msg.rFRA = 150;

    command_pub.publish(command_msg);



    ros::Duration(5).sleep();
    command_msg.rPRA = 70;
    command_pub.publish(command_msg);


    ros::Duration(5).sleep();
    command_msg.rPRA = 50;
    command_pub.publish(command_msg);



    ros::Duration(10).sleep();
  }


  return 0;
}
