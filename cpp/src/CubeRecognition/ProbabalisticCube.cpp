#include "ProbabalisticCube.hpp"

#include <cstdio>
#include <cstdlib>

#include <opencv2/opencv.hpp>

#include <SolverLib/FaceCube.hpp>

const ProbabalisticCube::PoseMatrix pose_prediction_uncertainty =
    ProbabalisticCube::PoseMatrix::eye() * 1.0;

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
    if (cubes.empty())
    {
        return;
    }

    { // Normalize using max log likelihood.
        const double max_log_likelihood =
            std::max_element(cubes.begin(), cubes.end(), compareCubeLikelihoods)->log_likelihood;
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
    child_template.pose_covariance += pose_prediction_uncertainty;

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
    std::vector<ProbabalisticCube> all_predictions;
    for (auto& cube : cubes)
    {
        const std::vector<ProbabalisticCube> predictions;
        for (const ProbabalisticCube& prediction : generatePredictions(cube))
        {
            all_predictions.push_back(prediction);
        }
    }
    return all_predictions;
}

template <class T, int R, int C>
double closeToEqual(const cv::Matx<T, R, C>& a, const cv::Matx<T, R, C>& b, T max_difference)
{
    for (int i = 0; i < R; ++i)
    {
        for (int j = 0; j < C; ++j)
        {
            if (std::abs(a(i, j) - b(i, j)) > max_difference)
            {
                return false;
            }
        }
    }
    return true;
}

void prune(std::vector<ProbabalisticCube>& cubes, size_t max_num)
{
    // Merge similar probability distributions
    std::multimap<std::string, ProbabalisticCube> merged_cubes;
    for (const ProbabalisticCube& cube : cubes)
    {
        const std::string cube_string = cube.cube_permutation.to_String();
        auto equal_range = merged_cubes.equal_range(cube_string);
        // Try to merge the cube with cubes from the equal_range, add new cube if not possible.
        auto match = [&]()
        {
            for (auto it = equal_range.first; it != equal_range.second; ++it)
            {
                const float max_difference = 1.0e-6;
                if (!closeToEqual(cube.pose_estimate, it->second.pose_estimate, max_difference))
                {
                    continue;
                }
                if (!closeToEqual(cube.pose_covariance, it->second.pose_covariance, max_difference))
                {
                    continue;
                }
                return it;
            }
            return equal_range.second;
        }();

        if (match != equal_range.second)
        {
            double max_log_likelihood = std::max(match->second.log_likelihood, cube.log_likelihood);
            match->second.log_likelihood = max_log_likelihood + log(
                exp(match->second.log_likelihood - max_log_likelihood) +
                exp(cube.log_likelihood - max_log_likelihood));
        }
        else
        {
            merged_cubes.emplace(cube.cube_permutation.to_String(), cube);
        }
    }

    printf("Removed %lu redundant cubes by merging\n", cubes.size() - merged_cubes.size());

    cubes.clear();
    for (const auto& it : merged_cubes)
    {
        cubes.push_back(it.second);
    }

    // Sort most likely first.
    std::sort(cubes.begin(), cubes.end(), compareCubeLikelihoodsReversed);
    if (cubes.size() > max_num)
    {
        cubes.resize(max_num);
    }
    normalizeLikelihood(cubes);
}
