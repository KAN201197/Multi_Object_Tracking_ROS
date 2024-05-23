#ifndef TRACK_HPP
#define TRACK_HPP

#include "object_tracking_ros/kalman_filter.hpp"
#include <Eigen/Dense>

class Track {
public:
    Track(int id, const Eigen::Vector4f &state);
    void predict();
    void update(const Eigen::Vector2f &measurement);
    Eigen::Vector4f getState() const;
    int getId() const;
    int getAge() const;
    int getTotalVisibleCount() const;
    int getConsecutiveInvisibleCount() const;
    void incrementInvisibleCount();
    void resetInvisibleCount();

private:
    int id_;
    int age_;
    int total_visible_count_;
    int consecutive_invisible_count_;
    KalmanFilter kf_;
};

#endif // TRACK_HPP
