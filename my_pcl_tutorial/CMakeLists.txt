cmake_minimum_required(VERSION 2.8.3)
project(my_pcl_tutorial)



set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  tf
  tf2 
  tf2_ros
  roscpp
  sensor_msgs
  iiwa_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(PCL 1.2 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need


add_message_files(
  FILES
  CModel_robot_input.msg
  CModel_robot_output.msg
)


generate_messages()

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)





###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(example_segmentation src/example_segmentation.cpp)
target_link_libraries(example_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(save_pcd src/save_pcd.cpp)
target_link_libraries(save_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(save_pcd_kinetic src/save_pcd_kinetic.cpp)
target_link_libraries(save_pcd_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alignment_continental src/alignment_continental.cpp)
target_link_libraries(alignment_continental ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alignment_continental_kinetic src/alignment_continental_kinetic.cpp)
target_link_libraries(alignment_continental_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(cluster_extraction_kinetic src/cluster_extraction_kinetic.cpp)
target_link_libraries(cluster_extraction_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cluster_crop_extraction_kinetic src/cluster_crop_extraction_kinetic.cpp)
target_link_libraries(cluster_crop_extraction_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})



add_executable(initialize_object_template_iiwa_pose src/initialize_object_template_iiwa_pose.cpp)
target_link_libraries(initialize_object_template_iiwa_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(initialize_object_template_iiwa_pose_kinetic src/initialize_object_template_iiwa_pose_kinetic.cpp)
target_link_libraries(initialize_object_template_iiwa_pose_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(initialize_object_template_kinetic src/initialize_object_template_kinetic.cpp)
target_link_libraries(initialize_object_template_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(initialize_iiwa_pose_kinetic src/initialize_iiwa_pose_kinetic.cpp)
target_link_libraries(initialize_iiwa_pose_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})




add_executable(transform_iiwa_pose src/transform_iiwa_pose.cpp)
target_link_libraries(transform_iiwa_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(transform_iiwa_pose_basedon_alignment src/transform_iiwa_pose_basedon_alignment.cpp)
target_link_libraries(transform_iiwa_pose_basedon_alignment ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(transform_iiwa_pose_basedon_alignment_kinetic src/transform_iiwa_pose_basedon_alignment_kinetic.cpp)
target_link_libraries(transform_iiwa_pose_basedon_alignment_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_one_transform_iiwa_pose_basedon_alignment_kinetic src/test_one_transform_iiwa_pose_basedon_alignment_kinetic.cpp)
target_link_libraries(test_one_transform_iiwa_pose_basedon_alignment_kinetic ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test_gripper src/test_gripper.cpp)
target_link_libraries(test_gripper ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test_3finger_gripper src/test_3finger_gripper.cpp)
target_link_libraries(test_3finger_gripper ${catkin_LIBRARIES} ${PCL_LIBRARIES})



add_executable(view_point_cloud src/view_point_cloud.cpp)
target_link_libraries(view_point_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(transform_iiwa_pose_grasp src/transform_iiwa_pose_grasp.cpp)
target_link_libraries(transform_iiwa_pose_grasp ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test_one_transform_iiwa_pose_grasp src/test_one_transform_iiwa_pose_grasp.cpp)
target_link_libraries(test_one_transform_iiwa_pose_grasp ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test_two_transform_iiwa_pose_grasp src/test_two_transform_iiwa_pose_grasp.cpp)
target_link_libraries(test_two_transform_iiwa_pose_grasp ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test_three_iiwa_ros src/test_three_iiwa_ros.cpp)
target_link_libraries(test_three_iiwa_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_three_transform_iiwa_pose_grasp src/test_three_transform_iiwa_pose_grasp.cpp)
target_link_libraries(test_three_transform_iiwa_pose_grasp ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test_four_transform_iiwa_pose_grasp src/test_four_transform_iiwa_pose_grasp.cpp)
target_link_libraries(test_four_transform_iiwa_pose_grasp ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_five_transform_iiwa_pose_grasp src/test_five_transform_iiwa_pose_grasp.cpp)
target_link_libraries(test_five_transform_iiwa_pose_grasp ${catkin_LIBRARIES} ${PCL_LIBRARIES})




## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_pcl_tutorial.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/my_pcl_tutorial_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_Bfind_package(PCL 1.2 REQUIRED)IN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_pcl_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
