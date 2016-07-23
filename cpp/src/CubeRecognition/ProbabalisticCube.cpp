#include "ProbabalisticCube.hpp"

#include <cstdio>
#include <cstdlib>

#include <opencv2/opencv.hpp>

#include <SolverLib/FaceCube.hpp>

double ProbabalisticCube::relativeLogLikelihoodOfRotations()
{
    double score = 0;
    for (int i = 3; i < D; ++i)
    {
        // The likelihood is rated as proportial to exp(-rotation^2)
        // thus promoting rotations closer to zero.
        score -= pose_estimate[i] * pose_estimate[i];
    }
    return score;
}

std::vector<ProbabalisticCube> generatePredictions(const ProbabalisticCube& parent)
{
    // Create a new cube pose distribution by applying possible moves (discrete permutations)
    // and rating the likelihood of them occuring.
    // The moves considered are:
    // * Whole cube rotations (+/-90 degrees, 3 axis = 6 moves)
    // * Face moves (6 faces, +/- 90 degrees = 12 moves)
    std::vector<ProbabalisticCube> children;
    for (int i = 0; i < 3; ++i)
    {
        for (int direction : {-1, 1})
        {
            { // Whole cube rotation
                ProbabalisticCube child = parent;
                child.pose_estimate[i + 3] -= M_PI_2 * direction;
                child.cube_permutation.moveAxis(i, direction, direction, direction);
                children.push_back(std::move(child));
            }
            { // Nearby side rotation
                ProbabalisticCube child = parent;
                child.pose_estimate[i + 6] -= M_PI_2 * direction;
                child.cube_permutation.moveAxis(i, direction, 0, 0);
                children.push_back(std::move(child));
            }
            { // Remote side rotation
                ProbabalisticCube child = parent;
                child.pose_estimate[i + 9] -= M_PI_2 * direction;
                child.cube_permutation.moveAxis(i, 0, 0, direction);
                children.push_back(std::move(child));
            }
        }
    }
    { // No rotation
        ProbabalisticCube child = parent;
        children.push_back(std::move(child));
    }

    { // Compute relative likelihood
        double max_log_likelihood = -std::numeric_limits<double>::infinity();
        for (ProbabalisticCube& child : children)
        {
            child.log_likelihood = child.relativeLogLikelihoodOfRotations();
            if (child.log_likelihood > max_log_likelihood)
            {
                max_log_likelihood = child.log_likelihood;
            }
        }
        // Normalize using max relative likelihood.
        for (ProbabalisticCube& child : children)
        {
            child.log_likelihood -= max_log_likelihood;
        }
    }

    { // Normalize according to parent likelihood
        double sum_likelihood = 0;
        for (ProbabalisticCube& child : children)
        {
            sum_likelihood += exp(child.log_likelihood);
        }
        double log_sum_likelihood = log(sum_likelihood);
        for (ProbabalisticCube& child : children)
        {
            child.log_likelihood -= log_sum_likelihood; // Divide by sum of children.
            child.log_likelihood += parent.log_likelihood; // Multiply with parent likelihood.
        }
    }

    return children;
}
