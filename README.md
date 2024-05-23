# Multi Object Tracking with Kalman Filter and Hungarian Algorithm in ROS

![2024-05-2316-36-08-ezgif com-video-to-gif-converter](https://github.com/KAN201197/Multi_Object_Tracking_ROS/assets/128454220/c1ab8b93-9f75-4b75-9718-89a975fa5b6d)

This repository contains a ROS package for multi-object tracking using a Kalman Filter for state estimation and the Hungarian Algorithm for data association. The package uses YOLOv3 for object detection and OpenCV for image processing.

## Installation

### Prerequisites

Ensure you have ROS installed on your system. This package has been tested with ROS Noetic. Additionally, you need to have OpenCV and Eigen installed.

```bash
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full
sudo apt-get install libopencv-dev
sudo apt-get install libeigen3-dev
```

### Cloning the Repository
Clone the repository into your catkin workspace and build it:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/multi_object_tracking_ros.git
cd ~/catkin_ws
catkin_make
```
