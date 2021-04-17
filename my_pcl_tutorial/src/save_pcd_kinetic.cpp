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
  const char* name = "workpiece";

  strcpy (filename,name);
  strcat (filename,".pcd");

  writer.write (filename, cloud, false);



  for(int i=0; i<=10; i++){

      strcpy (filename,name);

      char index[32];
      sprintf(index, "%d", i);

      strcat (filename,index);
      strcat (filename,".pcd");

      ifstream ifile(filename);
      if (!ifile){
          cout << "Press the ENTER key for workpiece" <<i <<".pcd"<<endl;

          if (cin.get() == '\n')
          {
//              strcpy (filename,name);

//              char index[32];
//              sprintf(index, "%d", i);

//              strcat (filename,index);
//              strcat (filename,".pcd");

              writer.write (filename, cloud, false);
              cout << "workpiece" <<i <<".pcd"<< " is saved"<<endl;

//              ifstream ifile(filename);
//              if (!ifile){
//                  writer.write (filename, cloud, false);
//                  cout << "workpiece" <<i <<".pcd"<< " is saved"<<endl;
//              }
          break;
          }

      }



  }


}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  
  //sensor_msgs::PointCloud2 output;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/sd/points", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1000);
  //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  
  ros::Rate loop_rate(10);

 
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while (ros::ok())
  {
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);
     cout<<count<<endl;

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    // std_msgs::String msg;

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    // chatter_pub.publish(msg);

    //pub.publish(output);

    // // get your point cloud message from somewhere
    // sensor_msgs::PointCloud2 cloud_msg2;

    // /// and publish the message
    // pub.publish(cloud_msg2);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;

  }

  return 0;
}
