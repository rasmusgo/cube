#pragma once

#include <tuple>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "Label.hpp"

void solveCamera(
    const std::vector<std::vector<Label>>& grouped_labels,
    const std::vector<std::vector<cv::Point2f>>& spatial_indices,
    const cv::Size& image_size);

std::pair<std::vector<std::vector<Label>>, std::vector<std::vector<cv::Point2f>>>
    connectLabels(const std::vector<Label>& labels);
