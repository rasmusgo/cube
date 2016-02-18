#pragma once

#include <opencv2/core/mat.hpp>

#include "LabelContour.hpp"

std::vector<cv::Point2f> findLabelPositions(const cv::Mat3b& img);
std::vector<cv::Point2f> findLabelPositions(const cv::Mat3b& img, double threshold);

std::vector<cv::Scalar> readLabelColors(const cv::Mat3b img, const std::vector<cv::Point2f>& points);

void drawLabelColors(cv::Mat3b& canvas,
    const std::vector<cv::Point2f>& points, const std::vector<cv::Scalar>& colors);

void drawLabelInfo(cv::Mat3b& canvas,
    const std::vector<cv::Point2f>& points,
    const std::vector<std::string>& texts,
    const cv::Scalar color,
    double font_scale);
