#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>

#include <iiwa_ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<iostream>
#include<fstream>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include<cmath>

using namespace tf;

int main (int argc, char **argv) {

  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init();

  std::string movegroup_name, ee_link;
  geometry_msgs::PoseStamped command_cartesian_position;

  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "manipulator");
  nh.param<std::string>("ee_link", ee_link, "iiwa_link_ee");

  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.5); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  int direction = 1;

  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;

  // Configure planner
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link);
  bool success_plan = false, motion_done = false, new_pose = false;


  // get the transform between camera and robot
  tf::TransformListener listener;
  tf::StampedTransform camera_robot_transform;
  try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform( "iiwa_link_0","tracking_origin",
                                    now, ros::Duration(3.0));
      listener.lookupTransform("iiwa_link_0","tracking_origin",
                             now, camera_robot_transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  cout<< "transformed translation x: "<<camera_robot_transform.getOrigin().x()<<endl;
  cout<< "transformed translation y: "<<camera_robot_transform.getOrigin().y()<<endl;
  cout<< "transformed translation z: "<<camera_robot_transform.getOrigin().z()<<endl;


  cout<< "transformed rotation x: "<<camera_robot_transform.getRotation().x()<<endl;
  cout<< "transformed rotation y: "<<camera_robot_transform.getRotation().y()<<endl;
  cout<< "transformed rotation z: "<<camera_robot_transform.getRotation().z()<<endl;
  cout<< "transformed rotation w: "<<camera_robot_transform.getRotation().w()<<endl;


  // assume the stamped vector in camera frame
  geometry_msgs::Vector3Stamped cv;
  Stamped< Vector3 > tf_cv;
  cv.vector.x = 0;
  cv.vector.y = 0.1;
  cv.vector.z = 0;
  cv.header.stamp = ros::Time();
  cv.header.frame_id = "tracking_origin";

  tf::vector3StampedMsgToTF(cv, tf_cv);


  // assume the stamped vector in robot frame
  geometry_msgs::Vector3Stamped rv;
  Stamped<Vector3> tf_rv;

  try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform( "iiwa_link_0","tracking_origin",
                                    now, ros::Duration(3.0));
      listener.lookupTransform("iiwa_link_0","tracking_origin",
                             now, camera_robot_transform);
      listener.transformVector("iiwa_link_0", cv, rv);
      cout<< "tf vector x: "<< rv.vector.x << endl;
      cout<< "tf vector y: "<< rv.vector.y << endl;
      cout<< "tf vector z: "<< rv.vector.z << endl;
      cout<< "tf distance is "<< sqrt(rv.vector.x*rv.vector.x + rv.vector.y*rv.vector.y + rv.vector.z*rv.vector.z) <<endl;

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }


  // assume the stamped rotation in camera



  // get the initial robot pose



  while (ros::ok()) {

    if (my_iiwa.getRobotIsConnected()) {

      command_cartesian_position = group.getCurrentPose(ee_link);

      //transform robot position using vector
      command_cartesian_position.pose.position.x = command_cartesian_position.pose.position.x + direction*rv.vector.x;
      command_cartesian_position.pose.position.y = command_cartesian_position.pose.position.y + direction*rv.vector.y;
      command_cartesian_position.pose.position.z = command_cartesian_position.pose.position.z + direction*rv.vector.z;


      // command_cartesian_position.pose.position.(x, y, z)
      // command_cartesian_position.pose.orientation.(x, y, z, w)


      group.setStartStateToCurrentState();
      group.setPoseTarget(command_cartesian_position, ee_link);
      success_plan = static_cast<bool>(group.plan(myplan));
      if (success_plan) {
        motion_done = static_cast<bool>(group.execute(myplan));
      }
      if (motion_done) {
        direction *= -1; // In the next iteration the motion will be on the opposite direction
        loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
      }

      ros::Duration(5.0).sleep(); // 5 seconds
    }
    else {
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
    }
  }
};
