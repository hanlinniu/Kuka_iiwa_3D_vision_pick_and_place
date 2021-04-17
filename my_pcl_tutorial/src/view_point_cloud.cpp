//http://pointclouds.org/documentation/tutorials/alignment_prerejective.php

#include <ros/ros.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
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
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


using namespace tf;

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_normals (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  PointCloudT::Ptr original_scene (new PointCloudT);
  PointCloudT::Ptr original_object (new PointCloudT);
  PointCloudT::Ptr transformed_original_object (new PointCloudT);
  PointCloudT::Ptr rotated_original_object (new PointCloudT);
  PointCloudT::Ptr rotated_translated_original_object (new PointCloudT);


  PointCloudT::Ptr scene_normals (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  // Get input object and scene
  if (argc != 2)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd\n", argv[0]);
    return (1);
  }

  // Load object and scene
  pcl::console::print_highlight ("Loading points clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0)
  {
    pcl::console::print_error ("Error loading object file!\n");
    return (1);
  }


  *original_scene = *object;


  // Show alignment
  pcl::visualization::PCLVisualizer visu0("Original Alignment");
  visu0.addPointCloud (original_scene, ColorHandlerT (original_scene, 0.0, 255.0, 255.0), "original_scene"); //it is blue color
  visu0.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_scene");

  visu0.spin ();


}
