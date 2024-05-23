#include "object_tracking_ros/track_manager.hpp"
#include <algorithm>

TrackManager::TrackManager() : next_id(0) {}

void TrackManager::predict() {
    for (auto &track : tracks) {
        track.predict();
    }
}

void TrackManager::update(const std::vector<Eigen::Vector2f> &measurements) {
    std::vector<std::vector<float>> cost_matrix = computeCostMatrix(measurements);
    if(cost_matrix.empty()) {
        ROS_INFO("COST MATRIX IS EMPTY");
    }
    HungarianAlgorithm hungarian;
    std::vector<int> assignment = hungarian.solve(cost_matrix);

    // Update tracks with assigned measurements
    for (size_t i = 0; i < tracks.size(); ++i) {
        if (assignment[i] != -1) {
            tracks[i].update(measurements[assignment[i]]);
        } else {
            tracks[i].incrementInvisibleCount();
        }
    }

    // Create new tracks for unassigned measurements
    for (size_t i = 0; i < measurements.size(); ++i) {
        if (std::find(assignment.begin(), assignment.end(), i) == assignment.end()) {
            float x = measurements[i](0, 0);
            float y = measurements[i](1, 0);
            tracks.emplace_back(next_id++, Eigen::Vector4f(x, y, 0, 0));
        }
    }

    // Remove old tracks
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(), [](const Track &track) {
        return track.getConsecutiveInvisibleCount() >= 5;
    }), tracks.end());
}

std::vector<Track> TrackManager::getTracks() const {
    return tracks;
}

std::vector<std::vector<float>> TrackManager::computeCostMatrix(const std::vector<Eigen::Vector2f> &measurements) {
    std::vector<std::vector<float>> cost_matrix(tracks.size(), std::vector<float>(measurements.size(), 0));
    for (size_t i = 0; i < tracks.size(); ++i) {
        for (size_t j = 0; j < measurements.size(); ++j) {
            Eigen::Vector4f state = tracks[i].getState();
            cost_matrix[i][j] = std::sqrt(std::pow(state(0) - measurements[j](0), 2) + std::pow(state(1) - measurements[j](1) , 2));
        }
    }
    return cost_matrix;
}
