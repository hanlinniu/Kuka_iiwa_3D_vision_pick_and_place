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
  if (argc != 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }

  // Load object and scene
  pcl::console::print_highlight ("Loading  scenepoint clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
      pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
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
  nest_object.setRadiusSearch (0.01);
  nest_object.setInputCloud (object);
  nest_object.compute (*object_normals);


  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*scene_normals);


  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.25);
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
  align.setMaximumIterations (20000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (10); // Number of nearest features to use
  align.setSimilarityThreshold (0.90f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (1.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.65f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

    // transform from Matrix4f to tf transform
    tf::Vector3 origin;
    origin.setValue(static_cast<double>(transformation(0,3)),static_cast<double>(transformation(1,3)),static_cast<double>(transformation(2,3)));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)),
          static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)),
          static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);

    double yaw, pitch, roll;
    transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 v = transform.getOrigin();

    //print transform in quaternion
    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
              << q.getZ() << ", " << q.getW() << "]" << std::endl
              << "             in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
              << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;



    Eigen::Matrix4f translation = transformation;

    translation(0,0) = 1;
    translation(0,1) = 0;
    translation(0,2) = 0;

    translation(1,0) = 0;
    translation(1,1) = 1;
    translation(1,2) = 0;

    translation(2,0) = 0;
    translation(2,1) = 0;
    translation(2,2) = 1;


    Eigen::Matrix4f rotation = transformation;

    rotation(0,3) = 0;
    rotation(1,3) = 0;
    rotation(2,3) = 0;



    // Show alignment
    pcl::visualization::PCLVisualizer visu("Original Alignment");
    visu.addPointCloud (original_scene, ColorHandlerT (original_scene, 0.0, 255.0, 255.0), "original_scene"); //it is blue color
    visu.addPointCloud (original_object, ColorHandlerT (original_object, 255.0, 0.0, 255.0), "original_object");//(object_aligned, ColorHandlerT (object_aligned, 255.0, 0.0, 255.0), "object_aligned");
    visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_scene");
    visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_object");


    // Apply an affine transform defined by an Eigen Transform.
    pcl::transformPointCloud (*original_object, *transformed_original_object, transformation);
    pcl::transformPointCloud (*original_object, *rotated_original_object, rotation);
    pcl::transformPointCloud (*rotated_original_object, *rotated_translated_original_object, translation);

    // Show alignment
    pcl::visualization::PCLVisualizer visu2("Matched Alignment");
    //visu.addPointCloud (original_scene, ColorHandlerT (original_scene, 0.0, 255.0, 255.0), "original_scene");
    //visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 255.0, 0.0, 255.0), "object_aligned");
    visu2.addPointCloud (original_scene, ColorHandlerT (original_scene, 0.0, 255.0, 255.0), "original_scene");//(object_aligned, ColorHandlerT (object_aligned, 255.0, 0.0, 255.0), "object_aligned");
    visu2.addPointCloud (transformed_original_object, ColorHandlerT (transformed_original_object, 255.0, 0.0, 255.0), "transformed_original_object");



    pcl::visualization::PCLVisualizer visu3("Rotated Matched Alignment");
    visu3.addPointCloud (original_scene, ColorHandlerT (original_scene, 0.0, 255.0, 255.0), "original_scene");//(object_aligned, ColorHandlerT (object_aligned, 255.0, 0.0, 255.0), "object_aligned");
    visu3.addPointCloud (rotated_original_object, ColorHandlerT (rotated_original_object, 255.0, 0.0, 255.0), "rotated_original_object");




    pcl::visualization::PCLVisualizer visu4("Rotated Translated Matched Alignment");

    visu4.addPointCloud (original_scene, ColorHandlerT (original_scene, 0.0, 255.0, 255.0), "original_scene");//(object_aligned, ColorHandlerT (object_aligned, 255.0, 0.0, 255.0), "object_aligned");
    visu4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_scene");
    visu4.addPointCloud (rotated_translated_original_object, ColorHandlerT (rotated_translated_original_object, 255.0, 0.0, 255.0), "rotated_translated_original_object");
    visu4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "rotated_translated_original_object");

    visu.spin ();
    visu2.spin ();
    visu3.spin ();
    visu4.spin();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }

  return (0);
}
