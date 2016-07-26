#pragma once

#include <opencv2/core/mat.hpp>

#include <SolverLib/FaceCube.hpp>

struct ProbabalisticCube
{
    double log_likelihood = 0;
    twophase::FaceCube cube_permutation;
    // The pose consists of
    // * position (x, y, z)
    // * rotation (U, R, F)
    // * side rotations (U R F D' L' B')
    static const int D = 12;
    using PoseEstimateVec   = cv::Vec<float, D>;
    using PoseCovarianceMat = cv::Matx<float, D, D>;
    PoseEstimateVec   pose_estimate{0, 0, 0};
    PoseCovarianceMat pose_covariance{PoseCovarianceMat::eye()};

    double relativeLogLikelihoodOfRotations();
};

void normalizeLikelihood(std::vector<ProbabalisticCube>& cubes);
std::vector<ProbabalisticCube> generatePredictions(const ProbabalisticCube& parent);
std::vector<ProbabalisticCube> predict(const std::vector<ProbabalisticCube>& cubes);
void prune(std::vector<ProbabalisticCube>& cubes, size_t max_num);