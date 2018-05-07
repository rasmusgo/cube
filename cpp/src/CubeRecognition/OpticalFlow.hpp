#pragma once

#include <opencv2/core/mat.hpp>

cv::Mat2f intermediateOpticalFlow(
    const cv::Mat3b& a,
    const cv::Mat3b& b);

cv::Mat3f colorizeOpticalFlow(const cv::Mat2f& flow);
