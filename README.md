# Kuka_iiwa_3D_vision_pick_and_place
This repository is for using 3D camera and Kuka iiwa robot arm to pick and place object


# for installing the kinect camera ros driver
https://github.com/code-iai/iai_kinect2


# for kinect camera calibration, the images data for calibration is under kinect_cal_data
useful link: https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration#calibrating-the-kinect-one


# To calibrate the relative position between kuka iiwa and kinect
1, roslaunch my_pcl_tutorial calibrate_iiwa_kinect.launch

# useful link for calibrating the relative position: https://github.com/IFL-CAMP/easy_handeye
 

# To view the aruco marker result
2, rosrun rqt_image_view rqt_image_view


# To initialize the original workpiece template and the original grasping point of kuka iiwa
3, roslaunch my_pcl_tutorial initialize_object_template_iiwa_pose_kinetic.launch 

3.1 By using the following command to save the workpiece_template_0.pcd as workpiece_template.pcd under directory ~/.ros

command: cp workpiece_template_0.pcd workpiece_template.pcd

3.2 manually control kuka iiwa to the unload position and save its pose as pose_unload.txt under directory ~/.ros

command: cp pose.txt pose_unload.txt

3.3 manually control kuka iiwa to the grasping position and save its pose as pose.txt under directory ~/.ros


# Use the following command to control kuka iiwa to grasp the randomly placed workpiece
4, roslaunch my_pcl_tutorial test_five_iiwa_test_transform_grasp.launch 