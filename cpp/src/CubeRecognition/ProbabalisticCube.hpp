#pragma once

#include <opencv2/core/mat.hpp>

#include <SolverLib/FaceCube.hpp>

#include "SolveCamera.hpp"

extern const cv::Matx33d urf_from_xyz;

struct ProbabalisticCube
{
    double log_likelihood = 0;
    twophase::FaceCube cube_permutation;
    // The pose consists of
    // * position (x, y, z)
    // * rotation (U, R, F)
    // * side rotations (U R F D' L' B')
    static const int D = 12;
    using PoseVector = cv::Vec<double, D>;
    using PoseMatrix = cv::Matx<double, D, D>;
    PoseVector pose_estimate{
        0, 0, 10, // position
        0, 0, 0,  // rotation
        0, 0, 0}; // first three side rotations
    PoseMatrix pose_covariance{PoseMatrix::eye() * 10};

    double relativeLogLikelihoodOfRotations();
};

inline bool compareCubeLikelihoods(const ProbabalisticCube& a, const ProbabalisticCube& b)

{
    return a.log_likelihood < b.log_likelihood;
}

inline bool compareCubeLikelihoodsReversed(const ProbabalisticCube& a, const ProbabalisticCube& b)
{
    return b.log_likelihood < a.log_likelihood;
}

void normalizeLikelihood(std::vector<ProbabalisticCube>& cubes);
std::vector<ProbabalisticCube> generatePredictions(const ProbabalisticCube& parent);
std::vector<ProbabalisticCube> predict(const std::vector<ProbabalisticCube>& cubes);
void prune(std::vector<ProbabalisticCube>& cubes, size_t max_num);

std::vector<cv::Point2f> projectCubeCorners(
    const Camera& calibrated_camera,
    const ProbabalisticCube& cube,
    float label_width);

void renderCoordinateSystem(
    cv::Mat3b& io_canvas,
    const Camera& calibrated_camera,
    const ProbabalisticCube& cube);
