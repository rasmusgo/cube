#pragma once

#include <tuple>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "Label.hpp"

std::pair<std::vector<std::vector<Label>>, std::vector<std::vector<cv::Point2f>>>
    connectLabels(const std::vector<Label>& labels);
