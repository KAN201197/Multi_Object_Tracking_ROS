#ifndef TRACK_MANAGER_HPP
#define TRACK_MANAGER_HPP

#include "object_tracking_ros/track.hpp"
#include "object_tracking_ros/hungarian_algorithm.hpp"
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>

class TrackManager {
public:
    TrackManager();
    void predict();
    void update(const std::vector<Eigen::Vector2f> &measurements);
    std::vector<Track> getTracks() const;

private:
    std::vector<Track> tracks;
    int next_id;    
    std::vector<std::vector<float>> computeCostMatrix(const std::vector<Eigen::Vector2f> &measurements);
};

#endif // TRACK_MANAGER_HPP
