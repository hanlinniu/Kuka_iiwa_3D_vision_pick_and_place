
// this code is for commanding the robot arm to 
// 1, move to unload position and standby
// 2, trigger the camera to capture the position of the workpiece and calculate out the transformed position
// 3. command the robot arm to the transformed position
// 4, command the robot arm to unload position and standby, and repeat the second step....

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

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/sd/points", 1, cloud_cb);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1000);

    ros::Rate loop_rate(10);


    int count = 0;
    int enter_index = 0;
    int exist_pcd = 0;


    //set up iiwa moveit
    iiwa_ros::iiwaRos my_iiwa;
    my_iiwa.init();
    my_iiwa.getPathParametersService().setJointRelativeVelocity(0.5); // The robot will now move at 50% velocity.


    std::string movegroup_name, ee_link;
    // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
    nh.param<std::string>("move_group", movegroup_name, "manipulator");
    nh.param<std::string>("ee_link", ee_link, "iiwa_link_ee");

    // Dynamic parameter to choose the rate at wich this node should run
    double ros_rate;
    nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
    ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

    int direction = 1;

    // Create MoveGroup
    moveit::planning_interface::MoveGroupInterface group(movegroup_name);
    moveit::planning_interface::MoveGroupInterface::Plan myplan;

    // Configure planner
    group.setPlanningTime(1);
    //group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlannerId(movegroup_name+"[RRTConnectkConfigDefault]");
    group.setEndEffectorLink(ee_link);
    bool success_plan = false, motion_done = false, new_pose = false;


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
          
          command_cartesian_position = group.getCurrentPose(ee_link);
          command_cartesian_position.pose.position.x = myArray_unload[0];
          command_cartesian_position.pose.position.y = myArray_unload[1];
          command_cartesian_position.pose.position.z = myArray_unload[2];
          command_cartesian_position.pose.orientation.x = myArray_unload[3];
          command_cartesian_position.pose.orientation.y = myArray_unload[4];
          command_cartesian_position.pose.orientation.z = myArray_unload[5];
          command_cartesian_position.pose.orientation.w = myArray_unload[6];

          group.setStartStateToCurrentState();
          group.setPoseTarget(command_cartesian_position, ee_link);
          success_plan = static_cast<bool>(group.plan(myplan));
           

          if (success_plan) 
          {

              cout<< "if you want move to unload position, type Y; otherwise, type N for replanning"<<endl;
              cin >> type;

              if (type == 'y')
              {
                  enter_index = 1;
                  motion_done = static_cast<bool>(group.execute(myplan));
                  if (motion_done) {
                    direction *= -1; // In the next iteration the motion will be on the opposite direction
                    break;
                  }
              }
              if (type == 'n')
              {
                  group.setStartStateToCurrentState();
                  group.setPoseTarget(command_cartesian_position, ee_link);
                  success_plan = static_cast<bool>(group.plan(myplan));
              }


          }
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
 




      // prepare the workpiece_test_0.pcd for alignment algorithm
       if( remove( "workpiece_test_0.pcd" ) != 0 )
           perror( "Error deleting file" );
       else
           puts( "workpiece_test_0.pcd File successfully deleted" );

       if( remove( "workpiece_test_scene.pcd" ) != 0 )
           perror( "Error deleting file" );
       else
           puts( "workpiece_test_scene.pcd File successfully deleted" );


      ros::Duration(1.0).sleep(); //wait for the kinect message come through
      ros::spinOnce();
      loop_rate.sleep();
      ++count;

      ifstream ifile("workpiece_test_scene.pcd");
      if (ifile)
      {
          cout << "workpiece_test_scene.pcd was saved!"<<endl;

          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

          // Load object and scene
          pcl::console::print_highlight ("Loading  scenepoint clouds...\n");
          if (pcl::io::loadPCDFile<pcl::PointXYZ> ("workpiece_test_scene.pcd", *cloud) < 0)
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
          writer.write<pcl::PointXYZ> ("workpiece_test_scene_downsampled.pcd", *cloud_filtered, false);

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

          writer.write<pcl::PointXYZ> ("workpiece_test_scene_downsampled_no_table.pcd", *cloud_f, false);


          pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
          Eigen::Vector4f min_pt(-0.3, -0.3, 0.5, 1);
          Eigen::Vector4f max_pt(0.3, 0.3, 2, 1);
          pcl::CropBox< pcl::PointXYZ> crop;
          crop.setInputCloud(cloud_filtered);
          crop.setMin(min_pt);
          crop.setMax(max_pt);
          crop.filter(*cropped_cloud);


          writer.write<pcl::PointXYZ> ("workpiece_test_scene_downsampled_no_table_Cropped.pcd", *cropped_cloud, false);


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

          int j = 0;
          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
          {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
              cloud_cluster->points.push_back (cropped_cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "workpiece_test_" << j << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            j++;
          }

          cout << "workpiece template is extraced as workpiece_test_0.pcd " <<endl;
  
      }

          // alignment algorithm: align the workpiece_test_0.pcd with workpiece_template.pcd and get the translation and rotation matrix
          PointCloudT::Ptr object (new PointCloudT);
          PointCloudT::Ptr object_normals (new PointCloudT);
          PointCloudT::Ptr object_aligned (new PointCloudT);
          PointCloudT::Ptr scene (new PointCloudT);
          PointCloudT::Ptr original_scene (new PointCloudT);
          PointCloudT::Ptr original_object (new PointCloudT);
          PointCloudT::Ptr transformed_original_object (new PointCloudT);
          PointCloudT::Ptr scene_normals (new PointCloudT);
          FeatureCloudT::Ptr object_features (new FeatureCloudT);
          FeatureCloudT::Ptr scene_features (new FeatureCloudT);


          // Load object test file and scene template file
          pcl::console::print_highlight ("Loading  scenepoint clouds...\n");
          if (pcl::io::loadPCDFile<PointNT> ("workpiece_test_0.pcd", *scene) < 0 ||
              pcl::io::loadPCDFile<PointNT> ("workpiece_template.pcd", *object) < 0)
          {
            pcl::console::print_error ("Error loading object/scene file!\n");
            return (1);
          }

          // Downsample
          *original_scene = *scene;
          *original_object = *object;


          pcl::console::print_highlight ("Downsampling...\n");
          pcl::VoxelGrid<PointNT> grid;
          const float leaf = 0.01f;
          grid.setLeafSize (leaf, leaf, leaf);
          grid.setInputCloud (object);
          grid.filter (*object);
          grid.setInputCloud (scene);
          grid.filter (*scene);


          // Estimate normals for object normals
          pcl::console::print_highlight ("Estimating object normals...\n");
          pcl::NormalEstimationOMP<PointNT,PointNT> nest_object;
          nest_object.setRadiusSearch (0.04);
          nest_object.setInputCloud (object);
          nest_object.compute (*object_normals);


          // Estimate normals for scene
          pcl::console::print_highlight ("Estimating scene normals...\n");
          pcl::NormalEstimationOMP<PointNT,PointNT> nest;
          nest.setRadiusSearch (0.04);
          nest.setInputCloud (scene);
          nest.compute (*scene_normals);


          // Estimate features
          pcl::console::print_highlight ("Estimating features...\n");
          FeatureEstimationT fest;
          fest.setRadiusSearch (0.30);
          fest.setInputCloud (object);
          fest.setInputNormals (object_normals);
          fest.compute (*object_features);
          fest.setInputCloud (scene);
          fest.setInputNormals (scene_normals);
          fest.compute (*scene_features);

          // Perform alignment
          pcl::console::print_highlight ("Starting alignment...\n");
          pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
          align.setInputSource (object);
          align.setSourceFeatures (object_features);
          align.setInputTarget (scene);
          align.setTargetFeatures (scene_features);
          align.setMaximumIterations (50000); // Number of RANSAC iterations
          align.setNumberOfSamples (4); // Number of points to sample for generating/prerejecting a pose
          align.setCorrespondenceRandomness (10); // Number of nearest features to use
          align.setSimilarityThreshold (0.90f); // Polygonal edge length similarity threshold
          align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
          align.setInlierFraction (0.80f); // Required inlier fraction for accepting a pose hypothesis
          {
            pcl::ScopeTime t("Alignment");
            align.align (*object_aligned);
          }

          ros::AsyncSpinner spinner(1);
          spinner.start();

          if (align.hasConverged ()){
              cout<< "Alignment succeed!\n"<<endl;
          }
          else
          {
            pcl::console::print_error ("Alignment failed!\n");

            return (1);
          }

          // Print results
          printf ("\n");
          Eigen::Matrix4f transformation = align.getFinalTransformation ();
          pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
          pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
          pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
          pcl::console::print_info ("\n");
          pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
          pcl::console::print_info ("\n");
          pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers().size (), object->size ());

          // transform from Matrix4f to tf vector3 translation
          tf::Vector3 origin;
          origin.setValue(static_cast<double>(transformation(0,3)),static_cast<double>(transformation(1,3)),static_cast<double>(transformation(2,3)));
          //cout << origin.getX()<<origin.getY()<<origin.getZ() << endl;

          // transform from Matrix4f to tf matrix3x3 rotation
          tf::Matrix3x3 tf3d;
          tf3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)),
                static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)),
                static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));

          tf::Quaternion tfqt;
          tf3d.getRotation(tfqt);

          // convert from Eigen::Matrix4f transformation to tf::Transform transform
          tf::Transform transform;
          transform.setOrigin(origin);   //from tf::Vector3 to tf translation
          transform.setRotation(tfqt);   //from tf::Transform to tf quaternion

          double yaw, pitch, roll;
          transform.getBasis().getRPY(roll, pitch, yaw); //from tf::Transform to roll pitch yaw radian angles
          tf::Quaternion q = transform.getRotation();
          tf::Vector3 v = transform.getOrigin();

          //print transform in quaternion
          std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
          std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                    << q.getZ() << ", " << q.getW() << "]" << std::endl
                    << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
                    << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;



         // if align.hasConverged(), calculate out the transformed position
          ros::spinOnce();
          loop_rate.sleep();
          //geometry_msgs::PoseStamped initial_cartesian_position; //initial pose under robot frame
          //geometry_msgs::PoseStamped command_cartesian_position; //command pose under robot frame
          geometry_msgs::PoseStamped initial_cartesian_position_camera_frame;  //initial pose under camera frame
          geometry_msgs::PoseStamped transformed_cartesian_position_camera_frame; //transformed pose under camera frame
          geometry_msgs::PoseStamped transformed_cartesian_position_robot_frame;

          // read the initial pose
          double myArray[7];
          ifstream initial_file("pose.txt");

          for(int i = 0; i < 7; ++i)
          {
              initial_file >> myArray[i];
          }

          initial_cartesian_position.pose.position.x = myArray[0];
          initial_cartesian_position.pose.position.y = myArray[1];
          initial_cartesian_position.pose.position.z = myArray[2]-0.2;
          initial_cartesian_position.pose.orientation.x = myArray[3];
          initial_cartesian_position.pose.orientation.y = myArray[4];
          initial_cartesian_position.pose.orientation.z = myArray[5];
          initial_cartesian_position.pose.orientation.w = myArray[6];

          initial_cartesian_position.header.frame_id = "iiwa_link_0";
          initial_cartesian_position.header.stamp = ros::Time::now();

          tf::TransformListener listener1;
          tf::StampedTransform rotated_camera_robot_transform;
          try{
              ros::Time now = ros::Time::now();
              listener1.waitForTransform( "tracking_origin","iiwa_link_0",
                                           now, ros::Duration(1.0));
              listener1.lookupTransform("tracking_origin","iiwa_link_0",
                                         now, rotated_camera_robot_transform);
             listener1.transformPose("tracking_origin", initial_cartesian_position, initial_cartesian_position_camera_frame);
           }
          catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
               //ros::Duration(1.0).sleep();
          }    

          static tf2_ros::StaticTransformBroadcaster static_broadcaster_zero;
          geometry_msgs::TransformStamped static_transformStamped_zero;

          static_transformStamped_zero.header.stamp = ros::Time::now();
          static_transformStamped_zero.header.frame_id = "tracking_origin";
          static_transformStamped_zero.child_frame_id = "initial_robot_pose";
          static_transformStamped_zero.transform.translation.x = initial_cartesian_position_camera_frame.pose.position.x ;
          static_transformStamped_zero.transform.translation.y = initial_cartesian_position_camera_frame.pose.position.y ;
          static_transformStamped_zero.transform.translation.z = initial_cartesian_position_camera_frame.pose.position.z ;
          static_transformStamped_zero.transform.rotation.x = initial_cartesian_position_camera_frame.pose.orientation.x;
          static_transformStamped_zero.transform.rotation.y = initial_cartesian_position_camera_frame.pose.orientation.y;
          static_transformStamped_zero.transform.rotation.z = initial_cartesian_position_camera_frame.pose.orientation.z;
          static_transformStamped_zero.transform.rotation.w = initial_cartesian_position_camera_frame.pose.orientation.w;
          static_broadcaster_zero.sendTransform(static_transformStamped_zero);



          static tf2_ros::StaticTransformBroadcaster static_broadcaster_one;
          geometry_msgs::TransformStamped static_transformStamped_one;
          geometry_msgs::Transform static_transform_one;
          tf::transformTFToMsg(transform, static_transform_one);
          static_transformStamped_one.header.stamp = ros::Time::now();
          static_transformStamped_one.header.frame_id = "tracking_origin";
           //static_transformStamped_one.child_frame_id = "transformed_test_robot_pose";
          static_transformStamped_one.transform = static_transform_one; 

           
           // pose in, transform pose out. both in camera frame
          transformed_cartesian_position_camera_frame.header.stamp = ros::Time::now();
          transformed_cartesian_position_camera_frame.header.frame_id = "tracking_origin";

          tf2::doTransform(initial_cartesian_position_camera_frame, transformed_cartesian_position_camera_frame, static_transformStamped_one);


          static tf2_ros::StaticTransformBroadcaster static_broadcaster_two;
          geometry_msgs::TransformStamped static_transformStamped_two;

          static_transformStamped_two.header.stamp = ros::Time::now();
          static_transformStamped_two.header.frame_id = "tracking_origin";
          static_transformStamped_two.child_frame_id = "transformed_robot_pose";
          static_transformStamped_two.transform.translation.x = transformed_cartesian_position_camera_frame.pose.position.x ;
          static_transformStamped_two.transform.translation.y = transformed_cartesian_position_camera_frame.pose.position.y ;
          static_transformStamped_two.transform.translation.z = transformed_cartesian_position_camera_frame.pose.position.z ;
          static_transformStamped_two.transform.rotation.x = transformed_cartesian_position_camera_frame.pose.orientation.x;
          static_transformStamped_two.transform.rotation.y = transformed_cartesian_position_camera_frame.pose.orientation.y;
          static_transformStamped_two.transform.rotation.z = transformed_cartesian_position_camera_frame.pose.orientation.z;
          static_transformStamped_two.transform.rotation.w = transformed_cartesian_position_camera_frame.pose.orientation.w;
          static_broadcaster_two.sendTransform(static_transformStamped_two);



          try{
              ros::Time now = ros::Time::now();
              listener1.waitForTransform( "tracking_origin","iiwa_link_0",
                                           now, ros::Duration(1.0));
              listener1.lookupTransform("tracking_origin","iiwa_link_0",
                                         now, rotated_camera_robot_transform);
              listener1.transformPose("iiwa_link_0", transformed_cartesian_position_camera_frame, transformed_cartesian_position_robot_frame);
          }
          catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
               //ros::Duration(1.0).sleep();
          }

          static tf2_ros::StaticTransformBroadcaster static_broadcaster_three;
          geometry_msgs::TransformStamped static_transformStamped_three;

          static_transformStamped_three.header.stamp = ros::Time::now();
          static_transformStamped_three.header.frame_id = "iiwa_link_0";
          static_transformStamped_three.child_frame_id = "transformed_robot_pose_robot_frame";
          static_transformStamped_three.transform.translation.x = transformed_cartesian_position_robot_frame.pose.position.x ;
          static_transformStamped_three.transform.translation.y = transformed_cartesian_position_robot_frame.pose.position.y ;
          static_transformStamped_three.transform.translation.z = transformed_cartesian_position_robot_frame.pose.position.z ;
          static_transformStamped_three.transform.rotation.x = transformed_cartesian_position_robot_frame.pose.orientation.x;
          static_transformStamped_three.transform.rotation.y = transformed_cartesian_position_robot_frame.pose.orientation.y;
          static_transformStamped_three.transform.rotation.z = transformed_cartesian_position_robot_frame.pose.orientation.z;
          static_transformStamped_three.transform.rotation.w = transformed_cartesian_position_robot_frame.pose.orientation.w;
          static_broadcaster_three.sendTransform(static_transformStamped_three);



          // moveit, using moveit to control the robot arm to the initial position and the transformed position
          cout<<"moveit begins"<<endl;
          double hold_altitude = 0.537;

          while (ros::ok())
          {

            if (my_iiwa.getRobotIsConnected())
              {

                ros::spinOnce();
                loop_rate.sleep();
                ros::AsyncSpinner spinner(1); 
                spinner.start();

                // go to initial position
                command_cartesian_position = group.getCurrentPose(ee_link);
                command_cartesian_position.pose.position.x = myArray[0];
                command_cartesian_position.pose.position.y = myArray[1];
                command_cartesian_position.pose.position.z = hold_altitude;
                command_cartesian_position.pose.orientation.x = myArray[3];
                command_cartesian_position.pose.orientation.y = myArray[4];
                command_cartesian_position.pose.orientation.z = myArray[5];
                command_cartesian_position.pose.orientation.w = myArray[6];

                group.setStartStateToCurrentState();
                group.setPoseTarget(command_cartesian_position, ee_link);
                success_plan = static_cast<bool>(group.plan(myplan));
                if (success_plan) 
                {

                    cout<< "if you want to move to initial position, type Y; otherwise, type N for replanning"<<endl;
                    cin >> type;

                    if (type == 'y')
                    {
                        enter_index = 1;
                        motion_done = static_cast<bool>(group.execute(myplan));
                        if (motion_done) {
                          break;
                        }
                    }
                    if (type == 'n')
                    {
                        group.setStartStateToCurrentState();
                        group.setPoseTarget(command_cartesian_position, ee_link);
                        success_plan = static_cast<bool>(group.plan(myplan));
                    }

                }
              }
              else
              {
               ROS_WARN_STREAM("Robot is not connected..."); 
               ros::Duration(2.0).sleep(); // 2 seconds
               //iiwa_ros::iiwaRos my_iiwa;
              // my_iiwa.init();
               ros::spinOnce();
               loop_rate.sleep();               
               ros::AsyncSpinner spinner(1); 
               spinner.start();
              }

          }

          
          while (ros::ok())
          {
            if (my_iiwa.getRobotIsConnected())
              {

                ros::spinOnce();
                loop_rate.sleep();
                ros::AsyncSpinner spinner(1); 
                spinner.start();
                // go to workpiece position with hold altitude
                //command_cartesian_position = group.getCurrentPose(ee_link);
                command_cartesian_position.pose.position.x = transformed_cartesian_position_robot_frame.pose.position.x;
                command_cartesian_position.pose.position.y = transformed_cartesian_position_robot_frame.pose.position.y;
                command_cartesian_position.pose.position.z = hold_altitude;
                command_cartesian_position.pose.orientation.x = transformed_cartesian_position_robot_frame.pose.orientation.x;
                command_cartesian_position.pose.orientation.y = transformed_cartesian_position_robot_frame.pose.orientation.y;
                command_cartesian_position.pose.orientation.z = transformed_cartesian_position_robot_frame.pose.orientation.z;
                command_cartesian_position.pose.orientation.w = transformed_cartesian_position_robot_frame.pose.orientation.w;


                group.setStartStateToCurrentState();
                group.setPoseTarget(command_cartesian_position, ee_link);
                success_plan = static_cast<bool>(group.plan(myplan));

                if (success_plan) 
                {
                    cout<< "if you want to move to transformed position, type Y; otherwise, type N for replanning"<<endl;
                    cin>>type;
                    if (type == 'y')
                    {
                        enter_index = 1;
                        motion_done = static_cast<bool>(group.execute(myplan));

                        if (motion_done) {

                          break;
                        }
                    }
                    if (cin.get() == 'n')
                    {
                        group.setStartStateToCurrentState();
                        group.setPoseTarget(command_cartesian_position, ee_link);
                        success_plan = static_cast<bool>(group.plan(myplan));
                    }
                }
              }
              else
              {
               ROS_WARN_STREAM("Robot is not connected..."); 
               ros::Duration(2.0).sleep(); // 2 seconds
               //iiwa_ros::iiwaRos my_iiwa;
               //my_iiwa.init();
               ros::spinOnce();
               loop_rate.sleep();
               ros::AsyncSpinner spinner(1); 
               spinner.start();
              }

          }



          


    }


}
