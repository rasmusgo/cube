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

/// Normalize likelihood of cubes so that the sum of likelihood is 1.0
void normalizeLikelihood(std::vector<ProbabalisticCube>& cubes)
{
    { // Normalize using max log likelihood.
        double max_log_likelihood = -std::numeric_limits<double>::infinity();
        for (ProbabalisticCube& cube : cubes)
        {
            if (cube.log_likelihood > max_log_likelihood)
            {
                max_log_likelihood = cube.log_likelihood;
            }
        }
        for (ProbabalisticCube& cube : cubes)
        {
            cube.log_likelihood -= max_log_likelihood;
        }
    }
    { // Normalize using sum of likelihood.
        double sum_likelihood = 0;
        for (ProbabalisticCube& cube : cubes)
        {
            sum_likelihood += exp(cube.log_likelihood);
        }
        double log_sum_likelihood = log(sum_likelihood);
        for (ProbabalisticCube& cube : cubes)
        {
            cube.log_likelihood -= log_sum_likelihood; // Divide by sum.
        }
    }
}

std::vector<ProbabalisticCube> generatePredictions(const ProbabalisticCube& parent)
{
    // Create a new cube pose distribution by applying possible moves (discrete permutations)
    // and rating the likelihood of them occuring.
    // The moves considered are:
    // * Whole cube rotations (+/-90 degrees, 3 axis = 6 moves)
    // * Face moves (6 faces, +/- 90 degrees = 12 moves)

    ProbabalisticCube child_template = parent;
    child_template.pose_covariance += ProbabalisticCube::PoseCovarianceMat::eye() * 0.1;

    std::vector<ProbabalisticCube> children;
    for (int i = 0; i < 3; ++i)
    {
        for (int direction : {-1, 1})
        {
            { // Whole cube rotation
                ProbabalisticCube child = child_template;
                child.pose_estimate[i + 3] -= M_PI_2 * direction;
                child.cube_permutation.moveAxis(i, direction, direction, direction);
                children.push_back(std::move(child));
            }
            { // Nearby side rotation
                ProbabalisticCube child = child_template;
                child.pose_estimate[i + 6] -= M_PI_2 * direction;
                child.cube_permutation.moveAxis(i, direction, 0, 0);
                children.push_back(std::move(child));
            }
            { // Remote side rotation
                ProbabalisticCube child = child_template;
                child.pose_estimate[i + 9] -= M_PI_2 * direction;
                child.cube_permutation.moveAxis(i, 0, 0, direction);
                children.push_back(std::move(child));
            }
        }
    }
    // No rotation
    children.push_back(child_template);

    // Compute relative likelihood
    for (ProbabalisticCube& child : children)
    {
        child.log_likelihood = child.relativeLogLikelihoodOfRotations();
    }

    // Normalize and multiply with parent likelihood
    normalizeLikelihood(children);
    for (ProbabalisticCube& child : children)
    {
        child.log_likelihood += parent.log_likelihood; // Multiply with parent likelihood.
    }
    return children;
}

std::vector<ProbabalisticCube> predict(const std::vector<ProbabalisticCube>& cubes)
{
    std::map<std::string, ProbabalisticCube> predictions_map;
    for (auto& cube : cubes)
    {
        for (const ProbabalisticCube& prediction : generatePredictions(cube))
        {
            std::string permutation_string = prediction.cube_permutation.to_String();
            auto it = predictions_map.find(permutation_string);
            if (it != predictions_map.end())
            {
                ProbabalisticCube& existing = it->second;
                // Normalize log likelihoods before exp() for numerical stability.
                double max_log_likelihood = std::max(existing.log_likelihood, prediction.log_likelihood);
                double relative_likelihood_a = exp(existing.log_likelihood - max_log_likelihood);
                double relative_likelihood_b = exp(prediction.log_likelihood - max_log_likelihood);
                double sum_relative_likelihood = relative_likelihood_a + relative_likelihood_b;
                double w_a = relative_likelihood_a / sum_relative_likelihood;
                double w_b = relative_likelihood_b / sum_relative_likelihood;
                ProbabalisticCube::PoseEstimateVec weighted_mean =
                    w_a * existing.pose_estimate + w_b * prediction.pose_estimate;
                // TODO(Rasmus): Check if a better formula for weighted coveriance is needed.
                ProbabalisticCube::PoseCovarianceMat weighted_covariance =
                    w_a * existing.pose_covariance + w_b * prediction.pose_covariance;
                existing.pose_estimate = weighted_mean;
                existing.pose_covariance = weighted_covariance;
                existing.log_likelihood = max_log_likelihood + log(sum_relative_likelihood);
            }
            else
            {
                predictions_map.insert({permutation_string, prediction});
            }
        }
    }
    std::vector<ProbabalisticCube> predictions_vector;
    for (auto& it : predictions_map)
    {
        predictions_vector.push_back(std::move(it.second));
    }
    return predictions_vector;
}

void prune(std::vector<ProbabalisticCube>& cubes, size_t max_num)
{
    std::sort(cubes.begin(), cubes.end(),
        [](const ProbabalisticCube& a, const ProbabalisticCube& b)
    {
        // Sort most likely first.
        return b.log_likelihood < a.log_likelihood;
    });
    if (cubes.size() > max_num)
    {
        cubes.resize(max_num);
    }
    normalizeLikelihood(cubes);
}
