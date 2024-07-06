# realsense_aruco
Package for running OpenCV ArUCo tag detection on Intel RealSense camera

<p align="middle">
  <img src="aruco_point.gif" width="45%">
  <img src="aruco_pose.gif" width="45%">
</p>

## Install
Clone repo
```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/apaik458/realsense_aruco.git
cd ~/catkin_ws && catkin_make
```
Install realsense-ros package
```bash
sudo apt-get install ros-noetic-realsense2-camera
```

## Usage
Start realsense nodes
```bash
roslaunch realsense2_camera rs_camera.launch
```
Run arucoPoint.py or arucoPose.py script to start 2D point or 6D pose detection
```bash
rosrun realsense_aruco arucoPoint.py
rosrun realsense_aruco arucoPose.py
```
Run test_arucoPoint.py or test_arucoPose.py script to view geometry_msgs Point or PoseStamped output
```bash
rosrun realsense_aruco test_arucoPoint.py
rosrun realsense_aruco test_arucoPose.py
```
