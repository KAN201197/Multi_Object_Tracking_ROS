#include "object_tracking_ros/multi_object_tracker.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_object_tracking");
    MultiObjectTracker multi_object_tracker;
    ros::spin();
    return 0;
}
