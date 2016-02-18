#pragma once

#include <tuple>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "LabelContour.hpp"

std::pair<std::vector<std::vector<LabelContour>>, std::vector<std::vector<cv::Point2f>>>
    connectLabelContours(const std::vector<LabelContour>& labels);
