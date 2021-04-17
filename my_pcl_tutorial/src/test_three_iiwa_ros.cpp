
// this code is for commanding the robot arm to move using iiwa_ros instead of using moveit, 
// this code is for preparing test_three_transform_iiwa_pose_grasp


#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>



#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>

#include <iiwa_ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<iostream>
#include<fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "tf/LinearMath/Transform.h"
#include<cmath>

#include <robotiq_s_model_control/SModel_robot_output.h>

using namespace tf;

ros::Publisher pub;


// Types for alignment
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


//sensor_msgs::PointCloud2 output;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  pcl::PCDWriter writer;

  char filename[80];
  const char* name = "workpiece_test_scene";

  strcpy (filename,name);
  strcat (filename,".pcd");

  writer.write (filename, cloud, false);

  ifstream ifile(filename);

  while (!ifile){

      cout << "Press the ENTER key for saving workpiece_test_scene.pcd"<<endl;

      if (cin.get() == '\n')
      {
          writer.write (filename, cloud, false);

          ifstream ifile(filename);

      }
  }

}


int main (int argc, char **argv) 

{

    // Initialize ROS
    ros::init (argc, argv, "CommandRobotMoveit");
    ros::NodeHandle nh;
    //ros::Duration(2.0).sleep();

    ros::Rate loop_rate(10);


    int count = 0;
    int enter_index = 0;
    int exist_pcd = 0;


    //set up iiwa moveit
    iiwa_ros::iiwaRos my_iiwa;
    my_iiwa.init();
    my_iiwa.getPathParametersService().setJointRelativeVelocity(0.1); // The robot will now move at 50% velocity.

    geometry_msgs::PoseStamped initial_cartesian_position; //initial pose under robot frame
    geometry_msgs::PoseStamped command_cartesian_position; //command pose under robot frame


    char type;

    while (ros::ok())
    {
      // control robot arm to the unload position

      ros::spinOnce();
      loop_rate.sleep();
      // read the unload pose
      double myArray_unload[7];
      ifstream file("pose_unload.txt");
      for(int i = 0; i < 7; ++i)
      {
          file >> myArray_unload[i];
          cout<< myArray_unload[i] <<endl;

      }

      while (ros::ok())
      {
        if (my_iiwa.getRobotIsConnected()) 

        {
          ros::spinOnce();
          loop_rate.sleep();
          //ros::Duration(2.0).sleep();
          ros::AsyncSpinner spinner(1); 
          spinner.start();
          
          //command_cartesian_position = group.getCurrentPose(ee_link);
          command_cartesian_position.pose.position.x = myArray_unload[0];
          command_cartesian_position.pose.position.y = myArray_unload[1];
          command_cartesian_position.pose.position.z = myArray_unload[2]-0.05;
          command_cartesian_position.pose.orientation.x = myArray_unload[3];
          command_cartesian_position.pose.orientation.y = myArray_unload[4];
          command_cartesian_position.pose.orientation.z = myArray_unload[5];
          command_cartesian_position.pose.orientation.w = myArray_unload[6];


          my_iiwa.setCartesianPose(command_cartesian_position);
          ros::Duration(3.0).sleep();
          break;
         
        }

        else
        {
          ROS_WARN_STREAM("Robot is not connected...");
          ros::Duration(2.0).sleep(); // 5 seconds
          //iiwa_ros::iiwaRos my_iiwa;
         // my_iiwa.init();
          ros::spinOnce();
          loop_rate.sleep();
          ros::AsyncSpinner spinner(1); 
          spinner.start();
        }

      }

       

      // read the unload pose
      double myArray[7];
      ifstream second_file("pose.txt");
      for(int i = 0; i < 7; ++i)
      {
          second_file >> myArray[i];
          cout<< myArray[i] <<endl;

      }

      while (ros::ok())
      {
        if (my_iiwa.getRobotIsConnected()) 

        {
          ros::spinOnce();
          loop_rate.sleep();
          //ros::Duration(2.0).sleep();
          ros::AsyncSpinner spinner(1); 
          spinner.start();

          cout<< "this is the second command"<<endl;
          
          //command_cartesian_position = group.getCurrentPose(ee_link);
          command_cartesian_position.pose.position.x = myArray[0];
          command_cartesian_position.pose.position.y = myArray[1];
          command_cartesian_position.pose.position.z = myArray[2]+0.2;
          command_cartesian_position.pose.orientation.x = myArray[3];
          command_cartesian_position.pose.orientation.y = myArray[4];
          command_cartesian_position.pose.orientation.z = myArray[5];
          command_cartesian_position.pose.orientation.w = myArray[6];

          cout<< "this is the second command 1"<<endl;

          my_iiwa.setCartesianPose(command_cartesian_position);
          ros::spinOnce();
          cout<< "this is the second command 2"<<endl;
          ros::Duration(3.0).sleep();
          break;

         
        }

        else
        {
          ROS_WARN_STREAM("Robot is not connected...");
          ros::Duration(2.0).sleep(); // 5 seconds
          //iiwa_ros::iiwaRos my_iiwa;
         // my_iiwa.init();
          ros::spinOnce();
          loop_rate.sleep();
          ros::AsyncSpinner spinner(1); 
          spinner.start();
        }

      }


      //ros::Duration(3.0).sleep(); 
 




     

    }


}
