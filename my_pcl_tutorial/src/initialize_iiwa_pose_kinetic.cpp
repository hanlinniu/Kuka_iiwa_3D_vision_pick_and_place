#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>


#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>

#include <iiwa_ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<iostream>
#include<fstream>

ros::Publisher pub;

int
main (int argc, char** argv)
{
   // delete pose.txt and workpiece_template_scene.pcd

    if( remove( "pose.txt" ) != 0 )
        perror( "Error deleting file" );
    else
        puts( "pose.txt File successfully deleted" );

    if( remove( "workpiece_template_scene.pcd" ) != 0 )
        perror( "Error deleting file" );
    else
        puts( "workpiece_template_scene.pcd File successfully deleted" );


//this part is to save workpiece_template_scene.pcd


  // Initialize ROS
  ros::init (argc, argv, "initialize_pose");
  ros::NodeHandle nh;

  
  // // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/sd/points", 1, cloud_cb);
  // // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1000);
  // //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  int enter_index = 0;

  // this part is to save the initialization pose of kuka iiwa
  cout << "Using control panel to move kuka iiwa to initilization pose" << endl;


  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);


  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init();

  std::string movegroup_name, ee_link;
  geometry_msgs::PoseStamped current_cartesian_position;
  geometry_msgs::PoseStamped command_cartesian_position;

  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "manipulator");
  nh.param<std::string>("ee_link", ee_link, "iiwa_link_ee");

  int direction = 1;

  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;

  // Configure planner
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink("iiwa_link_ee");
  bool success_plan = false, motion_done = false, new_pose = false;

  while (ros::ok()) 
  {
       if (my_iiwa.getRobotIsConnected()) 
       {

      cout << "Kuka iiwa is connected!" << endl;
      current_cartesian_position = group.getCurrentPose(ee_link);
      cout << "Kuka pose position is: " << current_cartesian_position.pose.position.x <<" " << current_cartesian_position.pose.position.y << " " << current_cartesian_position.pose.position.z << endl;
      cout << "Kuka pose orientation is: " << current_cartesian_position.pose.orientation.x<< " " <<current_cartesian_position.pose.orientation.y<< " " <<current_cartesian_position.pose.orientation.z<< " " <<current_cartesian_position.pose.orientation.w<< " " << endl;


      cout << "Press the ENTER key for saving kuka iiwa pose" << endl;


      if (cin.get() == '\n')
      {
          enter_index = 1;
      }

      while (enter_index==1)
      {
         ofstream myfile;
         myfile.open ("pose.txt");

         myfile << current_cartesian_position.pose.position.x <<" ";
         myfile << current_cartesian_position.pose.position.y <<" ";
         myfile << current_cartesian_position.pose.position.z <<" ";

         myfile << current_cartesian_position.pose.orientation.x <<" ";
         myfile << current_cartesian_position.pose.orientation.y <<" ";
         myfile << current_cartesian_position.pose.orientation.z <<" ";
         myfile << current_cartesian_position.pose.orientation.w <<" ";

         myfile.close();
         enter_index = 0;
         cout << "kuka iiwa pose is saved!" << endl;

//         ifstream file("pose.txt");


//         if(file.is_open())
//         {
//             double myArray[7];

//             for(int i = 0; i < 7; ++i)
//             {
//                 file >> myArray[i];
//                 cout << "saved pose value is " <<myArray[i] << endl;
//             }
//         }

      }

      cout << "kuka iiwa pose is saved to pose.txt, windows will be closed in 5 seconds!" << endl;
      ros::Duration(5.0).sleep();
       }

       else 
       {
         ROS_WARN_STREAM("Robot is not connected...");
         ros::Duration(5.0).sleep();
        } // 5 seconds
         ros::spinOnce();
         loop_rate.sleep();
         ros::AsyncSpinner spinner(1); 
         spinner.start();
  }




}
