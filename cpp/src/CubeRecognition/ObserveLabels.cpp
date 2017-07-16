#include "ObserveLabels.hpp"

#include <cstdio>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "LabelContour.hpp"
#include "SolveCamera.hpp"

std::vector<Camera> predictCameraPosesForLabel(
    const Camera& cam, const std::vector<cv::Point2f>& label_corners, float label_width)
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

    std::vector<Camera> cam_candidates;
    cam_candidates.reserve(candidate_points3d.size());
    for (const auto& points3d : candidate_points3d)
    {
        Camera new_cam = cam;
        cv::solvePnP(points3d, label_corners,
            cam.camera_matrix, cam.dist_coeffs,
            new_cam.rvec, new_cam.tvec);

        // Compute certainty of rotation and translation
        cv::Mat J;
        std::vector<cv::Point2f> p; // dummy output vector
        cv::projectPoints(points3d, new_cam.rvec, new_cam.tvec,
            new_cam.camera_matrix, new_cam.dist_coeffs, p, J);
        new_cam.JtJ = cv::Mat(J.t() * J, cv::Rect(0, 0, 6, 6));

        cam_candidates.push_back(std::move(new_cam));
    }
    return cam_candidates;
}

double scorePredictedCorners(const std::vector<cv::Point2f>& predicted_corners,
    const std::vector<std::vector<cv::Point2f>>& detected_corners)
{
    double score = 0;
    double sigma = 5;
    double inv_denom = 1.0 / (2 * sigma * sigma);
    for (int i = 0; i < predicted_corners.size(); i += 4)
    {
        for (const auto& corners : detected_corners)
        {
            // We are now looking at one predicted and one detected label.
            // Test different rotations (corner order) of the detected label since it
            // may not always match the predicted rotation.
            double label_score = 0;
            for (int rotation_i = 0; rotation_i < 4; ++rotation_i)
            {
                double rotation_score = 0;
                for (int corner_i = 0; corner_i < 4; ++corner_i)
                {
                    int j = (rotation_i + corner_i) % 4;
                    int k = (rotation_i + corner_i + 1) % 4;
                    double a_d2 = cv::norm(cv::Vec2f(predicted_corners[i + j] - corners[j]), cv::NORM_L2SQR);
                    double b_d2 = cv::norm(cv::Vec2f(predicted_corners[i + k] - corners[k]), cv::NORM_L2SQR);
                    rotation_score += exp(-a_d2 * inv_denom) * exp(-b_d2 * inv_denom);
                }
                label_score = std::max(label_score, rotation_score);
            }
            score += label_score;
        }
    }
    return score;
}

std::vector<double> scoreCameras(
    const std::vector<Camera>& all_camera_candidates,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width)
{
    std::vector<double> camera_scores;
    camera_scores.reserve(all_camera_candidates.size());
    for (const auto& cam : all_camera_candidates)
    {
        std::vector<cv::Point2f> predicted_corners = projectCubeCorners(cam, label_width);
        double score = scorePredictedCorners(predicted_corners, detected_corners);
        camera_scores.push_back(score);
    }

    return camera_scores;
}

void printCameraScores(const std::vector<double>& camera_scores)
{
    printf("Scored %lu cameras:\n", camera_scores.size());
    std::vector<double> sorted_camera_scores = camera_scores;
    std::sort(sorted_camera_scores.begin(), sorted_camera_scores.end());
    std::reverse(sorted_camera_scores.begin(), sorted_camera_scores.end());
    printf("Min score: %f max score: %f\n", sorted_camera_scores.back(), sorted_camera_scores.front());

    for (int i = 0; i < sorted_camera_scores.size() && i < 5; ++i)
    {
        double score = sorted_camera_scores[i];
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

void showCameraContributions(
    const std::vector<Camera>& all_camera_candidates,
    const std::vector<double>& camera_scores,
    float label_width, const cv::Mat3b& img)
{
    assert(all_camera_candidates.size() == camera_scores.size());
    cv::Mat1f accumulation(img.size(), 0.f);
    auto score_it = camera_scores.begin();
    for (const auto& cam : all_camera_candidates)
    {
        std::vector<cv::Point2f> predicted_corners = projectCubeCorners(cam, label_width);
        double score = *score_it++;
        accumulation += renderContribution(score, img.size(), predicted_corners);
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

    {
        std::vector<cv::Point> corners = {
            detected_corners[0],
            detected_corners[1],
            detected_corners[2],
            detected_corners[3],
        };
        cv::polylines(canvas, corners, true, cv::Scalar(255, 0, 255));
    }

    cv::imshow("predicted labels", canvas);
}

void showBestCameraCandidate(
    const std::vector<Camera>& all_camera_candidates,
    const std::vector<double>& camera_scores,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width, const cv::Mat3b& img)
{
    if (!all_camera_candidates.empty())
    {
        const size_t index = std::distance(camera_scores.begin(),
            std::max_element(camera_scores.begin(), camera_scores.end()));
        printf("detected_corners size: %lu index: %lu\n", detected_corners.size(), index);

        const Camera& cam = all_camera_candidates[index];
        std::vector<cv::Point2f> predicted_corners = projectCubeCorners(cam, label_width);

        // FIXME(Rasmus): Replace the hacky index / 9 with something more proper.
        showPredictedCorners(img, predicted_corners, detected_corners[index / 9]);
    }
}

double logNormPdf(const cv::Vec6d delta, const cv::Matx66d& JtJ)
{
    const double tau = 2.0 * M_PI;
    return -0.5 * ((delta.t() * JtJ * delta)[0] + std::log(cv::determinant(tau * JtJ.inv())));
}

ProbabalisticCube updateCube(const ProbabalisticCube& cube, const Camera& camera)
{
    using PoseMatrix = ProbabalisticCube::PoseMatrix;
    using PoseVector = ProbabalisticCube::PoseVector;
    const PoseMatrix cube_pose_information_matrix = cube.pose_covariance.inv();
    const PoseVector cube_pose_information_vector = cube_pose_information_matrix * cube.pose_estimate;

    const PoseMatrix camera_information_matrix =
        observed_space_from_state_space.t() * camera.JtJ * observed_space_from_state_space;
    const PoseVector camera_information_vector =
        observed_space_from_state_space.t() * camera.JtJ * cv::Vec6d(
        camera.rvec[0], camera.rvec[1], camera.rvec[2],
        camera.tvec[0], camera.tvec[1], camera.tvec[2]);

    // Add information from camera to cube pose.
    PoseMatrix updated_cube_pose_information_matrix =
        cube_pose_information_matrix + camera_information_matrix;
    PoseVector updated_cube_pose_information_vector =
        cube_pose_information_vector + camera_information_vector;

    ProbabalisticCube updated_cube = cube;
    updated_cube.pose_covariance = updated_cube_pose_information_matrix.inv();
    updated_cube.pose_estimate = updated_cube.pose_covariance * updated_cube_pose_information_vector;

    // FIXME(Rasmus): Make proper delta considering the coordinate systems of the cube and camera.
    const cv::Vec6d delta_in_camera(
        cube.pose_estimate[3] - camera.rvec[0],
        cube.pose_estimate[4] - camera.rvec[1],
        cube.pose_estimate[5] - camera.rvec[2],
        cube.pose_estimate[0] - camera.tvec[0],
        cube.pose_estimate[1] - camera.tvec[1],
        cube.pose_estimate[2] - camera.tvec[2]);

    cv::Matx66d cube_JtJ;
    for (int pose_i = 0; pose_i < 6; ++pose_i)
    {
        int cam_i = (pose_i + 3) % 6;
        for (int pose_j = 0; pose_j < 6; ++pose_j)
        {
            int cam_j = (pose_j + 3) % 6;
            cube_JtJ(cam_i, cam_j) = cube_pose_information_matrix(pose_i, pose_j);
        }
    }
    updated_cube.log_likelihood += logNormPdf(delta_in_camera, camera.JtJ);
    updated_cube.log_likelihood += logNormPdf(delta_in_camera, cube_JtJ);
    return updated_cube;
}

std::vector<ProbabalisticCube> update(
    const std::vector<ProbabalisticCube>& cube_hypotheses,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const Camera& calibrated_camera, float label_width, const cv::Mat3b& img)
{
    std::vector<Camera> all_camera_candidates;
    for (const auto& corners : detected_corners)
    {
        const std::vector<Camera> cameras =
            predictCameraPosesForLabel(calibrated_camera, corners, label_width);
        all_camera_candidates.insert(all_camera_candidates.end(), cameras.begin(), cameras.end());
    }

    if (all_camera_candidates.empty())
    {
        printf("No cameras predicted!\n");
        return cube_hypotheses;
    }

    // TODO(Rasmus): Merge camera predictions.

    const std::vector<double> camera_scores =
        scoreCameras(all_camera_candidates, detected_corners, label_width);

    printCameraScores(camera_scores);
    showCameraContributions(all_camera_candidates, camera_scores, label_width, img);

    showBestCameraCandidate(all_camera_candidates, camera_scores, detected_corners, label_width, img);

    std::vector<ProbabalisticCube> updated_cube_hypotheses;
    for (const auto& cube : cube_hypotheses)
    {
        for (const auto& camera : all_camera_candidates)
        {
            const ProbabalisticCube updated_cube = updateCube(cube, camera);
            updated_cube_hypotheses.push_back(updated_cube);
        }
    }
    normalizeLikelihood(updated_cube_hypotheses);

    return updated_cube_hypotheses;
}
