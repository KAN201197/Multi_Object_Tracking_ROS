#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    void init(const Eigen::Vector4f &state);
    void predict();
    void update(const Eigen::Vector2f &measurement);
    Eigen::Vector4f getState() const;

private:
    Eigen::Vector4f state_;
    Eigen::Matrix4f A_;  // State transition matrix
    Eigen::Matrix<float, 4, 2> B_;  // Control matrix
    Eigen::Matrix2f R_;  // Measurement noise covariance
    Eigen::Matrix4f Q_;  // Process noise covariance
    Eigen::Matrix4f P_;  // Error covariance matrix
    Eigen::Matrix<float, 2, 4> H_;  // Measurement matrix
    Eigen::Vector2f z_;  // Measurement vector
};

#endif // KALMAN_FILTER_HPP
