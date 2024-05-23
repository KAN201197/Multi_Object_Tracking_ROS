#include "object_tracking_ros/kalman_filter.hpp"

KalmanFilter::KalmanFilter() {
    A_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;
    B_ << 0.5, 0,
          0, 0.5,
          1, 0,
          0, 1;
    Q_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    H_ << 1, 0, 0, 0,
          0, 1, 0, 0;
    R_ << 1, 0,
          0, 1;
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
}

void KalmanFilter::init(const Eigen::Vector4f &state) {
    state_ = state;
}

void KalmanFilter::predict() {
    state_ = A_ * state_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector2f &measurement) {
    Eigen::Vector2f y = measurement - H_ * state_;
    Eigen::Matrix2f S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<float, 4, 2> K = P_ * H_.transpose() * S.inverse();
    state_ = state_ + K * y;
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    P_ = (I - K * H_) * P_;
}

Eigen::Vector4f KalmanFilter::getState() const {
    return state_;
}
