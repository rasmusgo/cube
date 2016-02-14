#pragma once

#include <opencv2/core/mat.hpp>

#include "Label.hpp"

using EdgeFunctionType = std::function<bool(const cv::Point&, const cv::Point&)>;

typedef float DT;
std::vector<Label> createlabels(cv::Size size, EdgeFunctionType& edge_function);
