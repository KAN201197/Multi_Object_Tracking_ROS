#include "object_tracking_ros/hungarian_algorithm.hpp"
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <iostream>

std::vector<int> HungarianAlgorithm::solve(const std::vector<std::vector<float>> &cost_matrix) {
    size_t rows = cost_matrix.size();
    size_t cols = rows > 0 ? cost_matrix[0].size() : 0;

    // Ensure the cost matrix is not empty and is rectangular
    if (rows == 0 || cols == 0) {
        // throw std::invalid_argument("Cost matrix must not be empty");
        return std::vector<int>();
    }
    for (const auto &row : cost_matrix) {
        if (row.size() != cols) {
            throw std::invalid_argument("All rows in the cost matrix must have the same number of columns");
        }
    }

    std::vector<int> assignment(rows, -1);
    std::vector<std::vector<float>> cost = cost_matrix;

    // Step 1: Subtract row minimum
    for (size_t i = 0; i < rows; ++i) {
        float row_min = *std::min_element(cost[i].begin(), cost[i].end());
        for (size_t j = 0; j < cols; ++j) {
            cost[i][j] -= row_min;
        }
    }

    // Step 2: Subtract column minimum
    for (size_t j = 0; j < cols; ++j) {
        float col_min = std::numeric_limits<float>::max();
        for (size_t i = 0; i < rows; ++i) {
            if (cost[i][j] < col_min) {
                col_min = cost[i][j];
            }
        }
        for (size_t i = 0; i < rows; ++i) {
            cost[i][j] -= col_min;
        }
    }

    // Initialize labels
    std::vector<float> label_row(rows, 0);
    std::vector<float> label_col(cols, 0);
    std::vector<int> match_col(rows, -1);
    std::vector<int> match_row(cols, -1);

    for (size_t i = 0; i < rows; ++i) {
        std::vector<float> slack(cols, std::numeric_limits<float>::max());
        std::vector<int> slack_row(cols, -1);
        std::vector<bool> s(rows, false);
        std::vector<bool> t(cols, false);
        int j0 = -1;
        int i0 = i;
        while (true) {
            s[i0] = true;
            float delta = std::numeric_limits<float>::max();
            int j1 = -1;
            for (size_t j = 0; j < cols; ++j) {
                if (!t[j]) {
                    float cur_slack = cost[i0][j] - label_row[i0] - label_col[j];
                    if (cur_slack < slack[j]) {
                        slack[j] = cur_slack;
                        slack_row[j] = i0;
                    }
                    if (slack[j] < delta) {
                        delta = slack[j];
                        j1 = j;
                    }
                }
            }
            for (size_t j = 0; j < cols; ++j) {
                if (t[j]) {
                    label_row[match_row[j]] += delta;
                    label_col[j] -= delta;
                } else {
                    slack[j] -= delta;
                }
            }
            label_row[i0] += delta;
            t[j1] = true;
            if (match_row[j1] == -1) {
                j0 = j1;
                break;
            } else {
                i0 = match_row[j1];
            }
        }

        while (j0 != -1) {
            int i1 = slack_row[j0];
            match_row[j0] = i1;
            std::swap(j0, match_col[i1]);
        }
    }

    for (size_t i = 0; i < rows; ++i) {
        assignment[i] = match_col[i];
    }

    return assignment;
}

bool HungarianAlgorithm::findUncoveredInMatrix(float item, const std::vector<std::vector<float>> &matrix, std::pair<int, int> &location) {
    for (size_t i = 0; i < matrix.size(); ++i) {
        for (size_t j = 0; j < matrix[i].size(); ++j) {
            if (matrix[i][j] == item) {
                location = {i, j};
                return true;
            }
        }
    }
    return false;
}

bool HungarianAlgorithm::pairInList(const std::pair<int, int> &item, const std::vector<std::pair<int, int>> &list) {
    return std::find(list.begin(), list.end(), item) != list.end();
}
