#ifndef MULTI_OBJECT_TRACKER_HPP
#define MULTI_OBJECT_TRACKER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "object_tracking_ros/track_manager.hpp"
#include <std_msgs/String.h>

class MultiObjectTracker {
public:
    MultiObjectTracker();
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    ros::Subscriber image_sub;
    ros::Publisher tracker_pub;
    TrackManager track_manager;
    cv::dnn::Net yolo_net;
    cv::dnn::Net fast_rcnn_net;

    std::vector<cv::Rect> detectObjects(const cv::Mat &frame);
    std::vector<cv::Rect> detectObjectsYOLO(const cv::Mat &frame);
    std::vector<cv::Rect> detectObjectsFastRCNN(const cv::Mat &frame);
    void publishTrackingInfo();
    std::vector<std::string> getOutputsNames(const cv::dnn::Net &net);
};

#endif // MULTI_OBJECT_TRACKER_HPP
