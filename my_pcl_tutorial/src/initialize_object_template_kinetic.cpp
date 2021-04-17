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
#include <pcl/visualization/pcl_visualizer.h>


#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>

#include <iiwa_ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<iostream>
#include<fstream>

ros::Publisher pub;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;

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
    //PointCloudT::Ptr cloud_cluster (new PointCloudT);
   //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
   

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
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/sd/points", 1, cloud_cb);
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
    if (ifile)
    {
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
        vg.setLeafSize (0.005f, 0.005f, 0.005f);
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
        seg.setDistanceThreshold (0.02);

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

        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Vector4f min_pt(-0.3, -0.3, 0.5, 1);
        Eigen::Vector4f max_pt(0.3, 0.3, 2, 1);
        pcl::CropBox< pcl::PointXYZ> crop;
        crop.setInputCloud(cloud_filtered);
        crop.setMin(min_pt);
        crop.setMax(max_pt);
        crop.filter(*cropped_cloud);


        writer.write<pcl::PointXYZ> ("workpiece_template_scene_downsampled_no_table_Cropped.pcd", *cropped_cloud, false);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cropped_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.06); // 2cm
        ec.setMinClusterSize (500);
        ec.setMaxClusterSize (10000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cropped_cloud);
        ec.extract (cluster_indices);

        int template_number = 0;
        int j = 0;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cropped_cloud->points[*pit]); //*
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
          std::stringstream ss;
          ss << "workpiece_template_" << j << ".pcd";
          writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
          template_number = template_number +1;
          j++;

          
        }
        
        cout << "workpiece template is saved as workpiece_template_0.pcd" <<endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_visual_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        PointCloudT::Ptr visual_cloud_cluster (new PointCloudT);
        pcl::io::loadPCDFile<pcl::PointXYZ> ("workpiece_template_0.pcd", *xyz_visual_cloud_cluster);
        pcl::copyPointCloud(*xyz_visual_cloud_cluster, *visual_cloud_cluster);

        int visual_indicator = 0;
        while ((ros::ok())&&(template_number>0))
        {
          pcl::visualization::PCLVisualizer visu("workpiece_template");
          visu.addPointCloud (xyz_visual_cloud_cluster, ColorHandlerT (xyz_visual_cloud_cluster, 0.0, 255.0, 255.0), "cloud_cluster");
          visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_cluster");
          visu.spinOnce();
          
          cout << "Press the ENTER key for saving workpiece_template.pcd"<<endl;

          if ((cin.get() == '\n') && (visual_indicator ==0))
          {
            
            std::stringstream ss;
            ss << "workpiece_template"<< ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *xyz_visual_cloud_cluster, false); 

            // this part is to save the initialization pose of kuka iiwa
            cout << "Workpiece template is initialized as workpiece_template.pcd!" << endl;
            cout << "Now you can use 'rosrun my_pcl_tutorial initialize_iiwa_pose_kinetic' to initialize the kuka iiwa pose" << endl;
            visual_indicator = 1;

          }
          visu.spin ();
        }
    
    }

  }
  exist_pcd=1;
  enter_index = 0;



}
