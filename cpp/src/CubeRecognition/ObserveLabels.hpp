#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "ProbabalisticCube.hpp"
#include "SolveCamera.hpp"

extern const cv::Matx<double, 6, 12> observed_space_from_state_space;

struct LabelObservation
{
    size_t label_index = 0;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    cv::Matx66d JtJ; // rvec, tvec
    float score = 0;
};

LabelObservation adjustedObservation(
    const LabelObservation& observation,
    const cv::Vec3d& target_rvec);

const LabelObservation& findBestObservation(const std::vector<LabelObservation>& observations);

void projectOuterCubeCornersWithUncertainties(
    const Camera& calibrated_camera,
    const LabelObservation& observation,
    std::vector<cv::Point2f>& out_points,
    std::vector<cv::Matx22f>& out_points_covariances);

void renderLabelObservation(
    cv::Mat3b& io_canvas,
    const Camera& calibrated_camera,
    const LabelObservation& observation,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width);

void showBestLabelObservation(
    const Camera& calibrated_camera,
    const std::vector<LabelObservation>& observations,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width, const cv::Mat3b& img);

std::vector<LabelObservation> generateObservations(
    const Camera& calibrated_camera,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const float label_width);

std::vector<LabelObservation> mergeObservationsPairwise(
    const Camera& calibrated_camera,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const std::vector<LabelObservation>& observations,
    float label_width);

std::vector<ProbabalisticCube> update(
    const std::vector<ProbabalisticCube>& cube_hypotheses,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const Camera& calibrated_camera, float label_width, const cv::Mat3b& img);
