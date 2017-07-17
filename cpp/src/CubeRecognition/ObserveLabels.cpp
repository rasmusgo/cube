#include "ObserveLabels.hpp"

#include <cstdio>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "LabelContour.hpp"
#include "SolveCamera.hpp"

const double corner_sigma = 5;
const double inv_corner_sigma_squared = 1.0 / (corner_sigma * corner_sigma);

std::vector<cv::Point2f> projectCubeCorners(
    const Camera& calibrated_camera,
    const LabelObservation& observation,
    float label_width)
{
    Camera cam = calibrated_camera;
    cam.rvec = observation.rvec;
    cam.tvec = observation.tvec;
    return projectCubeCorners(cam, label_width);
}

float scorePredictedCorners(const std::vector<cv::Point2f>& predicted_corners,
    const std::vector<std::vector<cv::Point2f>>& detected_corners)
{
    double score = 0;
    const double inv_denom = 1.0 / (2 * corner_sigma * corner_sigma);
    for (int i = 0; i < predicted_corners.size(); i += 4)
    {
        for (const auto& corners : detected_corners)
        {
            // We are now looking at one predicted and one detected label.
            // Test different rotations (corner order) of the detected label since it
            // may not always match the predicted rotation.
            double label_score = -std::numeric_limits<double>::infinity();
            for (int rotation_i = 0; rotation_i < 4; ++rotation_i)
            {
                double rotation_score = 0;
                for (int corner_i = 0; corner_i < 4; ++corner_i)
                {
                    int j = (rotation_i + corner_i) % 4;
                    int k = (rotation_i + corner_i + 1) % 4;
                    double a_d2 = cv::norm(cv::Vec2f(predicted_corners[i + j] - corners[j]), cv::NORM_L2SQR);
                    double b_d2 = cv::norm(cv::Vec2f(predicted_corners[i + k] - corners[k]), cv::NORM_L2SQR);
                    rotation_score += exp(-a_d2 * inv_denom + -b_d2 * inv_denom);
                }
                label_score = std::max(label_score, rotation_score);
            }
            score += label_score;
        }
    }
    return score;
}

std::vector<LabelObservation> generateObservationsForLabel(
    const Camera& calibrated_camera,
    const size_t label_index,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const float label_width)
{
    const float half_width = label_width * 0.5f;
    const std::vector<std::vector<cv::Point3f>> candidate_points3d = [&half_width]()
    {
        std::vector<std::vector<cv::Point3f>> point_groups;
        point_groups.reserve(9);
        for (float y : {-1, 0, 1})
        {
            for (float x : {-1, 0, 1})
            {
                std::vector<cv::Point3f> label_points3d;
                label_points3d.reserve(4);
                label_points3d.emplace_back(x + half_width, y + half_width, 1.5f);
                label_points3d.emplace_back(x - half_width, y + half_width, 1.5f);
                label_points3d.emplace_back(x - half_width, y - half_width, 1.5f);
                label_points3d.emplace_back(x + half_width, y - half_width, 1.5f);
                point_groups.push_back(std::move(label_points3d));
            }
        }
        return point_groups;
    }();

    std::vector<LabelObservation> observation_candidates;
    observation_candidates.reserve(candidate_points3d.size());
    for (const auto& points3d : candidate_points3d)
    {
        LabelObservation observation;
        observation.label_index = label_index;
        cv::solvePnP(points3d, detected_corners[label_index],
            calibrated_camera.camera_matrix, calibrated_camera.dist_coeffs,
            observation.rvec, observation.tvec);

        // Compute certainty of rotation and translation
        cv::Mat J;
        std::vector<cv::Point2f> p; // dummy output vector
        cv::projectPoints(points3d, observation.rvec, observation.tvec,
            calibrated_camera.camera_matrix, calibrated_camera.dist_coeffs, p, J);
        observation.JtJ = cv::Mat(J.t() * J, cv::Rect(0, 0, 6, 6));
        // covariance = JtJ.inv() * rmse^2
        // covariance.inv() = (JtJ.inv() * rmse^2).inv() = JtJ * (rmse^2).inv()
        observation.JtJ *= inv_corner_sigma_squared;

        const std::vector<cv::Point2f> predicted_corners =
            projectCubeCorners(calibrated_camera, observation, label_width);
        observation.score = scorePredictedCorners(predicted_corners, detected_corners);

        observation_candidates.push_back(std::move(observation));
    }
    return observation_candidates;
}

void printObservationScores(const std::vector<LabelObservation>& observations)
{
    printf("Scored %lu observations:\n", observations.size());
    std::vector<double> sorted_scores;
    sorted_scores.reserve(observations.size());
    for (const auto& observation : observations)
    {
        sorted_scores.push_back(observation.score);
    }
    std::sort(sorted_scores.begin(), sorted_scores.end());
    std::reverse(sorted_scores.begin(), sorted_scores.end());
    printf("Min score: %f max score: %f\n", sorted_scores.back(), sorted_scores.front());

    for (int i = 0; i < sorted_scores.size() && i < 5; ++i)
    {
        double score = sorted_scores[i];
        printf("#%d score: %f\n", i + 1, score);
    }
}

cv::Mat1f renderContribution(double score, cv::Size image_size,
    const std::vector<cv::Point2f>& predicted_corners)
{
    cv::Mat1f contribution(image_size, 0.f);
    for (int i = 0; i < predicted_corners.size(); i += 4)
    {
        std::vector<cv::Point> corners = {
            predicted_corners[i + 0],
            predicted_corners[i + 1],
            predicted_corners[i + 2],
            predicted_corners[i + 3],
        };
        cv::polylines(contribution, corners, true, cv::Scalar(score * score));
    }
    return contribution;
}

void showObservationContributions(
    const Camera& calibrated_camera,
    const std::vector<LabelObservation>& observations,
    float label_width, const cv::Mat3b& img)
{
    cv::Mat1f accumulation(img.size(), 0.f);
    for (const auto& observation : observations)
    {
        const std::vector<cv::Point2f> predicted_corners =
            projectCubeCorners(calibrated_camera, observation, label_width);
        accumulation += renderContribution(observation.score, img.size(), predicted_corners);
    }
    double minval;
    double maxval;
    cv::minMaxLoc(accumulation, &minval, &maxval);
    cv::imshow("accumulation", accumulation / maxval);
}

void showPredictedCorners(
    const cv::Mat3b& img,
    const std::vector<cv::Point2f>& predicted_corners,
    const std::vector<cv::Point2f>& detected_corners)
{
    cv::Mat3b canvas = img * 0.25f;
    for (int i = 0; i < predicted_corners.size(); i += 4)
    {
        std::vector<cv::Point> corners = {
            predicted_corners[i + 0],
            predicted_corners[i + 1],
            predicted_corners[i + 2],
            predicted_corners[i + 3],
        };
        cv::polylines(canvas, corners, true, cv::Scalar(0, 0, 255));
    }

    for (int i = 0; i < detected_corners.size(); i += 4)
    {
        std::vector<cv::Point> corners = {
            detected_corners[i + 0],
            detected_corners[i + 1],
            detected_corners[i + 2],
            detected_corners[i + 3],
        };
        cv::polylines(canvas, corners, true, cv::Scalar(255, 0, 255));
    }

    cv::imshow("predicted labels", canvas);
}

const LabelObservation& findBestObservation(const std::vector<LabelObservation>& observations)
{
    const LabelObservation& observation =
        *std::max_element(observations.begin(), observations.end(),
        [](const LabelObservation& a, const LabelObservation& b)
    {
        return a.score < b.score;
    });

    return observation;
}

void showBestLabelObservation(
    const Camera& calibrated_camera,
    const std::vector<LabelObservation>& observations,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width, const cv::Mat3b& img)
{
    if (!observations.empty())
    {
        const LabelObservation& observation = findBestObservation(observations);
        printf("detected_corners size: %lu label_index: %lu\n",
            detected_corners.size(), observation.label_index);

        const std::vector<cv::Point2f> predicted_corners =
            projectCubeCorners(calibrated_camera, observation, label_width);

        if (observation.label_index < detected_corners.size())
        {
            // Single index
            showPredictedCorners(img, predicted_corners, detected_corners[observation.label_index]);
        }
        else
        {
            // Multiple indices
            std::vector<cv::Point2f> selected_corners;
            for (size_t rest = observation.label_index; rest != 0; rest /= detected_corners.size())
            {
                const size_t index = rest % detected_corners.size();
                for (const auto corner : detected_corners[index])
                {
                    selected_corners.push_back(corner);
                }
            }
            showPredictedCorners(img, predicted_corners, selected_corners);
        }
    }
}

double logNormPdf(const cv::Vec6d delta, const cv::Matx66d& JtJ)
{
    const double tau = 2.0 * M_PI;
    return -0.5 * ((delta.t() * JtJ * delta)[0] + std::log(cv::determinant(tau * JtJ.inv())));
}

ProbabalisticCube updateCube(const ProbabalisticCube& cube, const LabelObservation& observation)
{
    using PoseMatrix = ProbabalisticCube::PoseMatrix;
    using PoseVector = ProbabalisticCube::PoseVector;
    const PoseMatrix cube_pose_information_matrix = cube.pose_covariance.inv();
    const PoseVector cube_pose_information_vector = cube_pose_information_matrix * cube.pose_estimate;

    const PoseMatrix observation_information_matrix =
        observed_space_from_state_space.t() * observation.JtJ * observed_space_from_state_space;
    const PoseVector observation_information_vector =
        observed_space_from_state_space.t() * observation.JtJ * cv::Vec6d(
        observation.rvec[0], observation.rvec[1], observation.rvec[2],
        observation.tvec[0], observation.tvec[1], observation.tvec[2]);

    // Add information from observation to cube pose.
    PoseMatrix updated_cube_pose_information_matrix =
        cube_pose_information_matrix + observation_information_matrix;
    PoseVector updated_cube_pose_information_vector =
        cube_pose_information_vector + observation_information_vector;

    ProbabalisticCube updated_cube = cube;
    updated_cube.pose_covariance = updated_cube_pose_information_matrix.inv();
    updated_cube.pose_estimate = updated_cube.pose_covariance * updated_cube_pose_information_vector;

    // FIXME(Rasmus): Make proper delta considering the different coordinate systems.
    const cv::Vec6d delta_in_observation(
        cube.pose_estimate[3] - observation.rvec[0],
        cube.pose_estimate[4] - observation.rvec[1],
        cube.pose_estimate[5] - observation.rvec[2],
        cube.pose_estimate[0] - observation.tvec[0],
        cube.pose_estimate[1] - observation.tvec[1],
        cube.pose_estimate[2] - observation.tvec[2]);

    cv::Matx66d cube_JtJ;
    for (int pose_i = 0; pose_i < 6; ++pose_i)
    {
        int obs_i = (pose_i + 3) % 6;
        for (int pose_j = 0; pose_j < 6; ++pose_j)
        {
            int obs_j = (pose_j + 3) % 6;
            cube_JtJ(obs_i, obs_j) = cube_pose_information_matrix(pose_i, pose_j);
        }
    }
    updated_cube.log_likelihood += logNormPdf(delta_in_observation, observation.JtJ);
    updated_cube.log_likelihood += logNormPdf(delta_in_observation, cube_JtJ);
    return updated_cube;
}

std::vector<LabelObservation> generateObservations(
    const Camera& calibrated_camera,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const float label_width)
{
    std::vector<LabelObservation> all_observations;
    for (size_t i = 0; i < detected_corners.size(); ++i)
    {
        const std::vector<LabelObservation> observations =
            generateObservationsForLabel(calibrated_camera, i, detected_corners, label_width);
        all_observations.insert(all_observations.end(), observations.begin(), observations.end());
    }

    if (all_observations.empty())
    {
        return {};
    }

    const float max_score = findBestObservation(all_observations).score;
    const float good_score = max_score * 0.5f;
    std::vector<LabelObservation> good_observations;
    for (const auto& observation : all_observations)
    {
        if (observation.score >= good_score)
        {
            good_observations.push_back(observation);
        }
    }

    printObservationScores(all_observations);
    printf("Selecting %lu of %lu observation candidates.\n",
        good_observations.size(), all_observations.size());

    return good_observations;
}

std::vector<ProbabalisticCube> update(
    const std::vector<ProbabalisticCube>& cube_hypotheses,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const Camera& calibrated_camera, float label_width, const cv::Mat3b& img)
{
    const std::vector<LabelObservation> observations =
        generateObservations(calibrated_camera, detected_corners, label_width);

    if (observations.empty())
    {
        printf("No observations!\n");
        return cube_hypotheses;
    }

    // Merge observations pairwise.
    std::vector<LabelObservation> merged_observations;
    for (size_t i = 0; i < observations.size(); ++i)
    {
        for (size_t j = i + 1; j < observations.size(); ++j)
        {
            const LabelObservation& a = observations[i];
            const LabelObservation& b = observations[j];
            if (a.label_index == b.label_index)
            {
                continue;
            }

            LabelObservation c;
            // Encode multiple label indices based on:
            // 1. A normal label_index is less than detected_corners.size()
            // 2. b.label_index > 0 because b.label_index > a.label_index.
            c.label_index = b.label_index * detected_corners.size() + a.label_index;

            c.JtJ = a.JtJ + b.JtJ;
            const cv::Vec6d a_vec(a.rvec[0], a.rvec[1], a.rvec[2], a.tvec[0], a.tvec[1], a.tvec[2]);
            const cv::Vec6d b_vec(b.rvec[0], b.rvec[1], b.rvec[2], b.tvec[0], b.tvec[1], b.tvec[2]);
            const cv::Vec6d c_vec = c.JtJ.inv() * (a.JtJ * a_vec + b.JtJ * b_vec);
            c.rvec << c_vec[0], c_vec[1], c_vec[2];
            c.tvec << c_vec[3], c_vec[4], c_vec[5];

            // Compute mahalanobis distance of c_vec in distribution of a and b.
            // Reject if c_vec is too unlikely.
            // Mahalanobis distance is symmetric for c_vec in a and b
            // because c_vec is the optimal combined estimate.
            const double max_mahalanobis_distance = 5.35; // Chi-square k=6, p=0.5
            //const double max_mahalanobis_distance = 12.59; // Chi-square k=6, p=0.05
            const double a_mahalanobis = cv::Mahalanobis(c_vec, a_vec, a.JtJ);
            if (a_mahalanobis > max_mahalanobis_distance)
            {
                continue;
            }

            const std::vector<cv::Point2f> predicted_corners =
                projectCubeCorners(calibrated_camera, c, label_width);
            c.score = scorePredictedCorners(predicted_corners, detected_corners);

            if (c.score > a.score &&
                c.score > b.score)
            {
                printf("Merging observations %lu and %lu (labels %lu and %lu): mahalanobis distance: %.2f scores: a: %.2f b: %.2f c: %.2f\n",
                    i, j, a.label_index, b.label_index, a_mahalanobis, a.score, b.score, c.score);

                merged_observations.push_back(c);
            }
        }
    }

    printf("Generated %lu observation pairs.\n", merged_observations.size());

    std::vector<LabelObservation> combined_observations = observations;
    combined_observations.insert(combined_observations.end(),
        merged_observations.begin(), merged_observations.end());

    showObservationContributions(calibrated_camera, combined_observations, label_width, img);
    showBestLabelObservation(calibrated_camera, combined_observations, detected_corners, label_width, img);

    // TODO(Rasmus): Add the hypotheses that all observation candidates are outliers.
    std::vector<ProbabalisticCube> updated_cube_hypotheses;
    for (const auto& cube : cube_hypotheses)
    {
        for (const auto& observation : combined_observations)
        {
            const ProbabalisticCube updated_cube = updateCube(cube, observation);
            updated_cube_hypotheses.push_back(updated_cube);
        }
    }
    normalizeLikelihood(updated_cube_hypotheses);

    return updated_cube_hypotheses;
}
