#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2D.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Initialize Kalman Filter Parameters
cv::KalmanFilter KF(4, 2, 0);
cv::Mat state(4, 1, CV_32F);
cv::Mat processNoise(4, 1, CV_32F);
cv::Mat measurement = cv::Mat::zeros(2, 1, CV_32F);

// ROS Publishers
ros::Publisher detection_pub;
ros::Publisher marker_pub;

// Initialize Kalman Filter
void initKalmanFilter() {
    KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                                                     0, 1, 0, 1,
                                                     0, 0, 1, 0,
                                                     0, 0, 0, 1);
    measurement.setTo(cv::Scalar(0));
    KF.statePre.at<float>(0) = 0;
    KF.statePre.at<float>(1) = 0;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(0.1));
}

void ImageCallBack(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
        cv::Mat frame;
        
        if (msg->encoding == "8UC3") {
            cv::cvtColor(cv_ptr->image, frame, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "rgb8") {
            cv::cvtColor(cv_ptr->image, frame, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "bgr8") {
            frame = cv_ptr->image;
        } else {
            ROS_ERROR("Unsupported image encoding: %s", msg->encoding.c_str());
            return;
        }

        cv::Mat mask;
        cv::inRange(frame, cv::Scalar(0, 0, 200), cv::Scalar(50, 50, 255), mask);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            auto largest_contour = std::max_element(contours.begin(), contours.end(),
                                                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                                                        return cv::contourArea(a) < cv::contourArea(b);
                                                    });
            cv::Rect bounding_box = cv::boundingRect(*largest_contour);

            // Update Kalman Filter
            measurement.at<float>(0) = bounding_box.x + bounding_box.width / 2;
            measurement.at<float>(1) = bounding_box.y + bounding_box.height / 2;

            cv::Mat prediction = KF.predict();
            cv::Mat estimated = KF.correct(measurement);

            cv::Point estimated_center(estimated.at<float>(0), estimated.at<float>(1));

            // Publish the detection
            vision_msgs::Detection2D detection;
            detection.bbox.center.x = estimated_center.x;
            detection.bbox.center.y = estimated_center.y;
            detection.bbox.size_x = bounding_box.width;
            detection.bbox.size_y = bounding_box.height;
            detection_pub.publish(detection);

            // Publish a visualization marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_frame";
            marker.header.stamp = ros::Time::now();
            marker.ns = "object_tracking";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = estimated_center.x;
            marker.pose.position.y = estimated_center.y;
            marker.pose.position.z = 0.0;
            marker.scale.x = bounding_box.width;
            marker.scale.y = bounding_box.height;
            marker.scale.z = 1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker_pub.publish(marker);

            // Draw the bounding box and center point on the frame
            cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, estimated_center, 5, cv::Scalar(255, 0, 0), -1);
        }

        // Show the image
        cv::imshow("Object Tracking", frame);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_tracking_node");
    ros::NodeHandle nh;

    initKalmanFilter();

    ros::Subscriber image_sub = nh.subscribe("/me5413/image_raw", 1, ImageCallBack);
    detection_pub = nh.advertise<vision_msgs::Detection2D>("object_detection", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::spin();

    return 0;
}
