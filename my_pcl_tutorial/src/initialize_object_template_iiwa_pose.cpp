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

//sensor_msgs::PointCloud2 output;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  pcl::PCDWriter writer;

  char filename[80];
  const char* name = "workpiece_template_scene";

  strcpy (filename,name);
  strcat (filename,".pcd");

  writer.write (filename, cloud, false);

  ifstream ifile(filename);

  while (!ifile){

      cout << "Press the ENTER key for saving workpiece_template_scene.pcd"<<endl;

      if (cin.get() == '\n')
      {
          writer.write (filename, cloud, false);
          ifstream ifile(filename);
      }

  }



}



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

  
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1000);
  //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  

  int count = 0;
  int enter_index = 0;
  int exist_pcd = 0;


  while (ros::ok())
  {
    ros::Duration(2.0).sleep(); //wait for the realsense message come through
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);


    ros::spinOnce();
    loop_rate.sleep();
    ++count;

    ifstream ifile("workpiece_template_scene.pcd");
    if (ifile){
        cout << "workpiece_template_scene.pcd was saved!"<<endl;


        //extract workpiece_templace.pcd here using cluster_extraction.cpp


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        // Load object and scene
        pcl::console::print_highlight ("Loading  scenepoint clouds...\n");
        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("workpiece_template_scene.pcd", *cloud) < 0)
        {
          pcl::console::print_error ("Error loading object file!\n");
          return (1);
        }


        // Read in the cloud data
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

        std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
        std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*



        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ> ("workpiece_template_scene_downsampled.pcd", *cloud_filtered, false);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.005);

        int i=0, nr_points = (int) cloud_filtered->points.size ();
        while (cloud_filtered->points.size () > 0.2 * nr_points)
        {
          // Segment the largest planar component from the remaining cloud
          seg.setInputCloud (cloud_filtered);
          seg.segment (*inliers, *coefficients);
          if (inliers->indices.size () == 0)
          {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
          }

          // Extract the planar inliers from the input cloud
          pcl::ExtractIndices<pcl::PointXYZ> extract;
          extract.setInputCloud (cloud_filtered);
          extract.setIndices (inliers);
          extract.setNegative (false);

          // Get the points associated with the planar surface
          extract.filter (*cloud_plane);
          std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

          // Remove the planar inliers, extract the rest
          extract.setNegative (true);
          extract.filter (*cloud_f);
          *cloud_filtered = *cloud_f;
        }

        writer.write<pcl::PointXYZ> ("workpiece_template_scene_downsampled_no_table.pcd", *cloud_f, false);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (3000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
          std::stringstream ss;
          ss << "workpiece_template_" << j << ".pcd";
          writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
          j++;
        }

        cout << "workpiece template is extraced as workpiece_template_0.pcd " <<endl;

        break;
    }

  }
  exist_pcd=1;
  enter_index = 0;


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

  while (ros::ok()) {
       if (my_iiwa.getRobotIsConnected()) {

      cout << "Kuka iiwa is connected!" << endl;
      current_cartesian_position = group.getCurrentPose(ee_link);


      //current_cartesian_position.pose.orientation.x
      //current_cartesian_position.pose.orientation.y
      //current_cartesian_position.pose.orientation.z
      //current_cartesian_position.pose.orientation.w

      //current_cartesian_position.pose.position.x
      //current_cartesian_position.pose.position.y
      //current_cartesian_position.pose.position.z




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

         ifstream file("pose.txt");


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

      cout << "kuka iiwa pose is saved to pose.txt, windows will be closed in 20 seconds!" << endl;

      ros::Duration(20.0).sleep();

       }

       else {
         ROS_WARN_STREAM("Robot is not connected...");
         ros::Duration(5.0).sleep();} // 5 seconds
  }
  ;




}
