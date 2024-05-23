#ifndef HUNGARIAN_ALGORITHM_HPP
#define HUNGARIAN_ALGORITHM_HPP

#include <vector>
#include <utility>

class HungarianAlgorithm {
public:
    std::vector<int> solve(const std::vector<std::vector<float>> &cost_matrix);

private:
    bool findUncoveredInMatrix(float item, const std::vector<std::vector<float>> &matrix, std::pair<int, int> &location);
    bool pairInList(const std::pair<int, int> &item, const std::vector<std::pair<int, int>> &list);
};

#endif // HUNGARIAN_ALGORITHM_HPP
