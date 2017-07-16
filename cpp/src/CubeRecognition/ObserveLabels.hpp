#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "ProbabalisticCube.hpp"
#include "SolveCamera.hpp"

std::vector<Camera> predictCameraPosesForLabel(
    const Camera& cam, const std::vector<cv::Point2f>& label_corners, float label_width);

double scorePredictedCorners(const std::vector<cv::Point2f>& predicted_corners,
    const std::vector<std::vector<cv::Point2f>>& detected_corners);

void showPredictedCorners(
    const cv::Mat3b& img,
    const std::vector<cv::Point2f>& predicted_corners,
    const std::vector<cv::Point2f>& detected_corners);

std::vector<double> scoreCameras(
    const std::vector<Camera>& all_camera_candidates,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width);

void printCameraScores(const std::vector<double>& camera_scores);

void showBestCameraCandidate(
    const std::vector<Camera>& all_camera_candidates,
    const std::vector<double>& camera_scores,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width, const cv::Mat3b& img);

std::vector<ProbabalisticCube> update(
    const std::vector<ProbabalisticCube>& cube_hypotheses,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const Camera& calibrated_camera, float label_width, const cv::Mat3b& img);
