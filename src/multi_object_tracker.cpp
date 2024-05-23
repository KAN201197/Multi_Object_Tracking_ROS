#include "object_tracking_ros/multi_object_tracker.hpp"

MultiObjectTracker::MultiObjectTracker() {
    ros::NodeHandle nh;
    image_sub = nh.subscribe("/me5413/image_raw", 1, &MultiObjectTracker::imageCallback, this);
    tracker_pub = nh.advertise<std_msgs::String>("/tracker/info", 1);

    // Load YOLO and Fast R-CNN models
    yolo_net = cv::dnn::readNetFromDarknet("/home/kent/catkin_ws/src/object_tracking_ros/config/yolov3.cfg", "/home/kent/catkin_ws/src/object_tracking_ros/config/yolov3.weights");
    yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // fast_rcnn_net = cv::dnn::readNetFromCaffe("/path/to/fast_rcnn.prototxt", "/path/to/fast_rcnn.caffemodel");
}

void MultiObjectTracker::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);

    cv::Mat frame;
        
    if (msg->encoding == "8UC3") {
        frame = cv_ptr->image;
    } else if (msg->encoding == "rgb8") {
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_BGR2RGB);
    } else if (msg->encoding == "bgr8") {
        frame = cv_ptr->image;
    } else {
        ROS_ERROR("Unsupported image encoding: %s", msg->encoding.c_str());
        return;
    }

    std::vector<cv::Rect> detections = detectObjects(frame);

    // std::vector<Eigen::Vector2f> measurement_vectors;
    // for (const auto &detection : detections) {
    //     measurement_vectors.emplace_back(detection.x + detection.width / 2, detection.y + detection.height / 2);
    // }

    // if (measurement_vectors.empty()) {
    //     ROS_WARN("No measurements to update.");
    //     return;
    // }

    // track_manager.predict();

    // if (track_manager.getTracks().empty()) {
    //     ROS_INFO("TRACKS EMPTY");
    // }

    // track_manager.update(measurement_vectors);

    // for (const auto &track : track_manager.getTracks()) {
    //     Eigen::Vector4f state = track.getState();
    //     cv::rectangle(frame, cv::Rect(state(0) - 20, state(1) - 20, 40, 40), cv::Scalar(0, 255, 0), 2);
    // }

    // Draw bounding boxes and labels for detected objects
    for (size_t i = 0; i < detections.size(); ++i) {
        cv::Rect detection = detections[i];

        // Draw bounding box
        cv::rectangle(frame, detection, cv::Scalar(0, 255, 0), 2);

        // Add label or ID near the bounding box
        std::string label = "Object " + std::to_string(i + 1);
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.5;
        int thickness = 1;
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, font, font_scale, thickness, &baseline);
        cv::Point text_org(detection.x + (detection.width - text_size.width) / 2, detection.y - 5);
        cv::putText(frame, label, text_org, font, font_scale, cv::Scalar(0, 0, 255), thickness);
    }

    cv::imshow("Tracking", frame);
    cv::waitKey(1);

    publishTrackingInfo();
}

std::vector<cv::Rect> MultiObjectTracker::detectObjects(const cv::Mat &frame) {
    std::vector<cv::Rect> yolo_detections = detectObjectsYOLO(frame);
    // std::vector<cv::Rect> fast_rcnn_detections = detectObjectsFastRCNN(frame);

    // Combine detections
    // std::vector<cv::Rect> detections;
    // detections.insert(detections.end(), yolo_detections.begin(), yolo_detections.end());
    // detections.insert(detections.end(), fast_rcnn_detections.begin(), fast_rcnn_detections.end());
    return yolo_detections;
}

std::vector<cv::Rect> MultiObjectTracker::detectObjectsYOLO(const cv::Mat &frame) {
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
    yolo_net.setInput(blob);
    std::vector<cv::Mat> outs;
    yolo_net.forward(outs, getOutputsNames(yolo_net));

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    for (const auto &out : outs) {
        for (int i = 0; i < out.rows; ++i) {
            float *data = (float*)out.data + i * out.cols;
            float confidence = data[4];
            if (confidence > 0.5) {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                boxes.emplace_back(left, top, width, height);
                confidences.push_back(confidence);
            }
        }
    }
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);
    std::vector<cv::Rect> result;
    for (auto i : indices) {
        result.push_back(boxes[i]);
    }
    return result;
}

// std::vector<cv::Rect> MultiObjectTracker::detectObjectsFastRCNN(const cv::Mat &frame) {
//     cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(600, 600), cv::Scalar(104.0, 177.0, 123.0));
//     fast_rcnn_net.setInput(blob);
//     cv::Mat detections = fast_rcnn_net.forward();

//     std::vector<cv::Rect> boxes;
//     for (int i = 0; i < detections.size[2]; ++i) {
//         float confidence = detections.at<float>(0, 0, i, 2);
//         if (confidence > 0.5) {
//             int left = static_cast<int>(detections.at<float>(0, 0, i, 3) * frame.cols);
//             int top = static_cast<int>(detections.at<float>(0, 0, i, 4) * frame.rows);
//             int right = static_cast<int>(detections.at<float>(0, 0, i, 5) * frame.cols);
//             int bottom = static_cast<int>(detections.at<float>(0, 0, i, 6) * frame.rows);
//             boxes.emplace_back(left, top, right - left, bottom - top);
//         }
//     }
//     return boxes;
// }

void MultiObjectTracker::publishTrackingInfo() {
    std_msgs::String msg;
    std::stringstream ss;

    for (const auto &track : track_manager.getTracks()) {
        Eigen::Vector4f state = track.getState();
        ss << "Track ID: " << track.getId() << " Position: (" << state(0) << ", " << state(1) << ")\n";
    }

    msg.data = ss.str();
    tracker_pub.publish(msg);
}

std::vector<std::string> MultiObjectTracker::getOutputsNames(const cv::dnn::Net &net) {
    static std::vector<std::string> names;
    if (names.empty()) {
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        std::vector<std::string> layersNames = net.getLayerNames();
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i) {
            names[i] = layersNames[outLayers[i] - 1];
        }
    }
    return names;
}
