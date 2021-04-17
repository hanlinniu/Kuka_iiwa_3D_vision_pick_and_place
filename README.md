# Kuka_iiwa_3D_vision_pick_and_place
This repository is for using 3D camera and Kuka iiwa robot arm to pick and place object. You can use realsense camera or Kinect v2 camera.
This work is implemented in paper [3D Vision-guided Pick-and-Place Using Kuka LBR iiwa Robot](https://ieeexplore.ieee.org/abstract/document/9382674) published in 2021 IEEE/SICE International Symposium on System Integration (SII)

Demo Video 1: [Link](https://youtu.be/M7GtTTIV9rA) 


Kinect V2 camera intrinsic before and after calibration:

![](intrinsic_calibration.gif)


Eye to hand calibration:

![](eye_to_hand_calibration.gif)


Before and after segmenting point cloud:

![](segmenting_point_cloud.gif)


Point cloud matching using ICP:

![](point_cloud_matching.gif)


Final grasping demo:

![](grasping_demo.gif)

### setting up kuka iiwa package: [Link](https://github.com/IFL-CAMP/iiwa_stack/wiki)
### for installing the kinect camera ros driver: [Link](https://github.com/code-iai/iai_kinect2)
### installing easy hand eye package: [Link](https://github.com/IFL-CAMP/easy_handeye)
### installing robotiq gripper driver: [Link](https://github.com/ros-industrial/robotiq)

### necessary library:
```
sudo apt-cache search ros-kinetic-control*
sudo apt-get install ros-kinetic-control-toolbox
sudo ap install ros-kinetic-soem
sudo apt install ros-kinetic-soem
sudo apt-get install ros-kinetic-controller-interface
sudo apt-get install ros-kinetic-controller-manager
sudo apt-get install ros-kinetic-joint-limits-interface
sudo apt-get install ros-kinetic-socketcan-interface
sudo apt-get install ros-kinetic-moveit-core
sudo apt-get install ros-kinetic-moveit-ros-planning
sudo apt-get install ros-kinetic-moveit-ros-planning-interface
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-moveit
```

### for kinect v2 camera, you need intrinsic calibration, the images data for calibration is under kinect_cal_data: [Link](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration#calibrating-the-kinect-one)


## Git clone this project 

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/hanlinniu/Kuka_iiwa_3D_vision_pick_and_place.git
$ cd ~/catkin_ws && catkin_make
```

### To calibrate the relative position between kuka iiwa and kinect v2 camera
```
roslaunch my_pcl_tutorial calibrate_iiwa_kinect.launch
```
### To calibrate the relative position between kuka iiwa and realsense camera
```
roslaunch my_pcl_tutorial calibrate_iiwa_realsense.launch
```

### To view the aruco marker result
```
rosrun rqt_image_view rqt_image_view
```

### To initialize the original workpiece template and the original grasping point of kuka iiwa
```
roslaunch my_pcl_tutorial initialize_object_template_iiwa_pose_kinetic.launch 
```

### By using the following command to save the workpiece_template_0.pcd as workpiece_template.pcd under directory ~/.ros
```
cp workpiece_template_0.pcd workpiece_template.pcd
```
### manually control kuka iiwa to the unload position and save its pose as pose_unload.txt under directory ~/.ros
```
cp pose.txt pose_unload.txt
```
### manually control kuka iiwa to the grasping position and save its pose as pose.txt under directory ~/.ros

### Use the following command to control kuka iiwa to grasp the randomly placed workpiece
```
roslaunch my_pcl_tutorial test_five_iiwa_test_transform_grasp.launch 
```
### Paper
If you use this code in your research, please cite our SII paper:
```
@inproceedings{niu20213d,
  title={3D Vision-guided Pick-and-Place Using Kuka LBR iiwa Robot},
  author={Niu, Hanlin and Ji, Ze and Zhu, Zihang and Yin, Hujun and Carrasco, Joaquin},
  booktitle={2021 IEEE/SICE International Symposium on System Integration (SII)},
  pages={592--593},
  year={2021},
  organization={IEEE}
}
}
