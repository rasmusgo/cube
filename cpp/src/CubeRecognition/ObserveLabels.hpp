#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "ProbabalisticCube.hpp"
#include "SolveCamera.hpp"

struct LabelObservation
{
    float score = 0;
    //cv::Vec6d pose_vector; // rvec, tvec
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    cv::Matx66d JtJ;
};

std::vector<LabelObservation> generateObservationsForLabel(
    const Camera& calibrated_camera,
    const std::vector<cv::Point2f>& label_corners,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width);

std::vector<ProbabalisticCube> update(
    const std::vector<ProbabalisticCube>& cube_hypotheses,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const Camera& calibrated_camera, float label_width, const cv::Mat3b& img);
