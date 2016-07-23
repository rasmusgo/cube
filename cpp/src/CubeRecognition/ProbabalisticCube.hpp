#pragma once

#include <opencv2/core/mat.hpp>

#include <SolverLib/FaceCube.hpp>

struct ProbabalisticCube
{
    double log_likelihood;
    twophase::FaceCube cube_permutation;
    // The pose consists of
    // * position (x, y, z)
    // * rotation (U, R, F)
    // * side rotations (U R F D' L' B')
    static const int D = 12;
    cv::Vec<float, D>     pose_estimate;
    cv::Matx<float, D, D> pose_covariance;

    double relativeLogLikelihoodOfRotations();

};

std::vector<ProbabalisticCube> generatePredictions(const ProbabalisticCube& parent);
