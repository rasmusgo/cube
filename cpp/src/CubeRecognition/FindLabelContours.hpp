#pragma once

#include <opencv2/core/mat.hpp>

#include "LabelContour.hpp"

using EdgeFunctionType = std::function<bool(const cv::Point&, const cv::Point&)>;

typedef float DT;
std::vector<LabelContour> findLabelContours(cv::Size size, EdgeFunctionType& edge_function);
