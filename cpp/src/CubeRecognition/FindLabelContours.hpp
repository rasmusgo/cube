#pragma once

#include <opencv2/core/mat.hpp>

#include "LabelContour.hpp"

using EdgeFunctionType = std::function<bool(const cv::Point&, const cv::Point&)>;

std::vector<LabelContour> findLabelContours(const cv::Mat3b& img, double threshold, bool visualize);
std::vector<LabelContour> findLabelContours(cv::Size size, EdgeFunctionType& edge_function, bool visualize);

std::vector<std::vector<cv::Point2f>> findLabelCorners(const std::vector<LabelContour>& labels);

void drawLabel(cv::Mat& canvas, const LabelContour& label, const cv::Scalar& color);
void drawLabels(cv::Mat& canvas, const std::vector<LabelContour>& labels, const cv::Scalar& color);

void showDetectedLabels(
    const cv::Mat3b& img,
    const std::vector<LabelContour>& labels,
    const std::vector<std::vector<cv::Point2f>>& detected_corners);
