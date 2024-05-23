#include "object_tracking_ros/track.hpp"

Track::Track(int id, const Eigen::Vector4f &state)
    : id_(id), age_(0), total_visible_count_(0), consecutive_invisible_count_(0) {
    kf_.init(state);
}

void Track::predict() {
    kf_.predict();
}

void Track::update(const Eigen::Vector2f &measurement) {
    kf_.update(measurement);
    total_visible_count_++;
    consecutive_invisible_count_ = 0;
}

Eigen::Vector4f Track::getState() const {
    return kf_.getState();
}

int Track::getId() const {
    return id_;
}

int Track::getAge() const {
    return age_;
}

int Track::getTotalVisibleCount() const {
    return total_visible_count_;
}

int Track::getConsecutiveInvisibleCount() const {
    return consecutive_invisible_count_;
}

void Track::incrementInvisibleCount() {
    consecutive_invisible_count_++;
}

void Track::resetInvisibleCount() {
    consecutive_invisible_count_ = 0;
}
