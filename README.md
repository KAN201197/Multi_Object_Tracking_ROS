# Multi Object Tracking with Kalman Filter and Hungarian Algorithm in ROS

![2024-05-2316-36-08-ezgif com-video-to-gif-converter](https://github.com/KAN201197/Multi_Object_Tracking_ROS/assets/128454220/c1ab8b93-9f75-4b75-9718-89a975fa5b6d)

This repository contains a ROS package for multi-object tracking using a Kalman Filter for state estimation and the Hungarian Algorithm for data association. The package uses YOLOv3 for object detection and OpenCV for image processing.

## Dependencies

- ROS Noetic
- OpenCV
- Eigen
- cv_bridge
- image_transport
- sensor_msgs
- std_msgs

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
git clone https://github.com/KAN201197/Multi_Object_Tracking_ROS.git
cd ~/catkin_ws
catkin_make
```

## Usage
### Running the Object Tracking Node

To launch the object tracking node, use the following command:

```bash
roslaunch object_tracking_ros object_tracking.launch
```
## Nodes

### object_tracking_ros_node
This node subscribes to raw image data, performs object detection using YOLOv3, and tracks the detected objects using a Kalman Filter and Hungarian Algorithm.

**Subscribed Topics**

/me5413/image_raw: The raw image data from the camera.

**Published Topics**

/tracker/info: Information about the tracked objects.

## Configuration File

- **config/yolov3.cfg**: YOLOv3 configuration file.
- **config/yolov3.weights**: YOLOv3 weights file.

## Code Structure
### Header File
- **kalman_filter.hpp**: Defines the KalmanFilter class for Kalman Filter Implementation.
- **track.hpp**: Defines the Track class for representing individual tracked objects.
- **track_manager.hpp**: Defines the TrackManager class for managing multiple tracks.
- **hungarian_algorithm.hpp**: Implementation of the Hungarian Algorithm for solving the assignment problem.
- **multi_object_tracker.hpp**: Defines the MultiObjectTracker class for handling image data, detecting objects, and tracking them.

### Source Code File
- **kalman_filter.cpp**: Implementation of the KalmanFilter class.
- **track.cpp**: Implementation of the Track class.
- **track_manager.cpp**: Implementation of the TrackManager class.
- **hungarian_algorithm.cpp**: Implementation of the Hungarian Algorithm.
- **multi_object_tracker.cpp**: Implementation of the MultiObjectTracker class.


