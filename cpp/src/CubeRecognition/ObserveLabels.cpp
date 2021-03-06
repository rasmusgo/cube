#include "ObserveLabels.hpp"

#include <cstdio>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "LabelContour.hpp"
#include "MatrixTools.hpp"
#include "SolveCamera.hpp"

const double corner_sigma = 5;
const double inv_corner_sigma_squared = 1.0 / (corner_sigma * corner_sigma);

// observation.JtJ has order: rotation, position
// cube pose has order: position, rotation, side rotations
const cv::Matx<double, 6, 12> camera_space_from_state_space = // = J_observation_wrt_state
    *(cv::Matx<double, 6, 12>() <<
    0, 0, 0,  1, 0, 0,  0, 0, 0, 0, 0, 0,
    0, 0, 0,  0, 1, 0,  0, 0, 0, 0, 0, 0,
    0, 0, 0,  0, 0, 1,  0, 0, 0, 0, 0, 0,
    1, 0, 0,  0, 0, 0,  0, 0, 0, 0, 0, 0,
    0, 1, 0,  0, 0, 0,  0, 0, 0, 0, 0, 0,
    0, 0, 1,  0, 0, 0,  0, 0, 0, 0, 0, 0);

ProbabalisticCube::PoseVector poseEstimateFromObservation(const LabelObservation& observation)
{
    // Add a small prior to be able to solve for the full cube pose.
    const ProbabalisticCube::PoseMatrix prior = ProbabalisticCube::PoseMatrix::eye() * 1.0e-9;
    return (observation.information_matrix + prior).inv() * observation.information_vector;
}

std::vector<cv::Point2f> projectCubeCorners(
    const Camera& calibrated_camera,
    const LabelObservation& observation,
    float label_width)
{
    const ProbabalisticCube prior;
    const ProbabalisticCube cube = updateCube(prior, observation);
    return projectCubeCorners(calibrated_camera, cube, label_width);
}

void renderCoordinateSystem(
    cv::Mat3b& io_canvas,
    const Camera& calibrated_camera,
    const LabelObservation& observation)
{
    const ProbabalisticCube prior;
    const ProbabalisticCube cube = updateCube(prior, observation);
    renderCoordinateSystem(io_canvas, calibrated_camera, cube);
}

ProbabalisticCube::PoseMatrix expandMatx33toPoseMatrix(const cv::Matx33d& mat33)
{
    using PoseMatrix = ProbabalisticCube::PoseMatrix;
    PoseMatrix full_mat = PoseMatrix::eye();
    cv::Mat(mat33).copyTo(cv::Mat(cv::Mat(full_mat), cv::Rect(3, 3, 3, 3)));
    // FIXME(Rasmus): Shuffle face rotations
    return full_mat;
}

LabelObservation adjustedObservation(
    const LabelObservation& observation,
    const cv::Matx33d& adjustment)
{
    using PoseMatrix = ProbabalisticCube::PoseMatrix;
    using PoseVector = ProbabalisticCube::PoseVector;
    LabelObservation adj_obs = observation;

    // Adjust pose.
    const PoseVector observation_estimate = poseEstimateFromObservation(observation);
    const cv::Vec3d observation_rvec(observation_estimate[3], observation_estimate[3], observation_estimate[3]);
    cv::Matx33d obs_space_from_world;
    cv::Rodrigues(observation_rvec, obs_space_from_world);
    const cv::Matx33d adjusted_obs_space_from_world = obs_space_from_world * adjustment;
    cv::Vec3d adjusted_obs_rvec;
    cv::Rodrigues(adjusted_obs_space_from_world, adjusted_obs_rvec);

    const cv::Matx<double, 6,3> urfdlb_from_xyz = *(cv::Matx<double, 6,3>() <<
        0, 1, 0,
        -1, 0, 0,
        0, 0, 1,
        0, -1, 0,
        1, 0, 0,
        0, 0, -1);

    const cv::Matx66d side_rotations_adjustment =
        urfdlb_from_xyz * adjustment * urfdlb_from_xyz.t();
    const cv::Vec6d observation_side_rotations = cv::Mat1d(cv::Mat1d(observation_estimate), cv::Rect(0, 6, 1, 6));
    const cv::Vec6d adjusted_side_rotations = side_rotations_adjustment * observation_side_rotations;

    //(J69_side_rotations_wrt_rmat.reshape<18,3>() * adjustment).reshape<6,9>() * J96_adj_rmat_wrt_adj_side_rotations;

    // FIXME(Rasmus): This goes wrong sometimes. Could be related to singularities in rvec form.
    // Adjust information matrix
    const cv::Matx<double, 3, 9> J39_rvec_wrt_rmat = rodriguesJacobian(obs_space_from_world);
    const cv::Matx<double, 9, 3> J93_adj_rmat_wrt_adj_rvec = rodriguesJacobian(adjusted_obs_rvec);
    const PoseMatrix A = expandMatx33toPoseMatrix(
        (J39_rvec_wrt_rmat.reshape<9,3>() * adjustment).reshape<3,9>() *
        J93_adj_rmat_wrt_adj_rvec);
    adj_obs.information_matrix = A.t() * observation.information_matrix * A;

    return adj_obs;
}

LabelObservation adjustedObservation(
    const LabelObservation& observation,
    const cv::Vec3d& target_rvec)
{
    // Adjust rotation of observation to match target using 90 degree increments.
    cv::Matx33d target_from_world;
    cv::Rodrigues(target_rvec, target_from_world);

    const ProbabalisticCube::PoseVector pose = poseEstimateFromObservation(observation);
    const cv::Vec3d observation_rvec(pose[3], pose[3], pose[3]);
    cv::Matx33d obs_space_from_world;
    cv::Rodrigues(observation_rvec, obs_space_from_world);

    //                            target_from_world ~= obs_space_from_world * adjustment
    // obs_space_from_world.t() * target_from_world ~=                        adjustment
    const cv::Matx33d raw_adjustment = obs_space_from_world.t() * target_from_world;

    cv::Matx33d adjustment = closest90DegreeRotation(raw_adjustment);
    if (cv::determinant(adjustment) != 1.0)
    {
        const double raw_determinant = cv::determinant(raw_adjustment);
        const double determinant = cv::determinant(adjustment);
        printf("WARNING: adjustment has a bad determinant of %f, skipping adjustment.\n",
            determinant);
        printf("raw_adjustment: (determinant: %f)\n", raw_determinant);
        printMatx33d(raw_adjustment);
        printf("adjustment: (determinant: %f)\n", determinant);
        printMatx33d(adjustment);

        return observation;
    }
    if (adjustment == cv::Matx33d::eye())
    {
        return observation;
    }

    return adjustedObservation(observation, adjustment);
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

void adjustRotationVectorAndPoints(cv::Vec3d& io_rvec, std::vector<cv::Point3f>& io_points)
{
    cv::Matx33d rmat;
    cv::Rodrigues(io_rvec, rmat);
    const cv::Matx33d adjustment = closest90DegreeRotation(rmat);
    rmat = rmat * adjustment.t();
    cv::Rodrigues(rmat, io_rvec);
    for (auto& p : io_points)
    {
        p = cv::Matx33f(adjustment) * p;
    }
}

cv::Matx66d estimatePoseCertainty(
    const Camera& calibrated_camera,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const std::vector<cv::Point3f>& points)
{
    cv::Mat J;
    std::vector<cv::Point2f> p; // dummy output vector
    cv::projectPoints(points, rvec, tvec,
        calibrated_camera.camera_matrix, calibrated_camera.dist_coeffs, p, J);
    cv::Matx66d JtJ = cv::Mat(J.t() * J, cv::Rect(0, 0, 6, 6));
    // covariance = JtJ.inv() * rmse^2
    // covariance.inv() = (JtJ.inv() * rmse^2).inv() = JtJ * (rmse^2).inv()
    JtJ *= inv_corner_sigma_squared;
    return JtJ;
}

cv::Point3f pointsAverage(const std::vector<cv::Point3f>& points)
{
    assert(!points.empty());
    cv::Point3d sum(0, 0, 0);
    for (const auto& p : points)
    {
        sum += cv::Point3d(p);
    }
    return sum / double(points.size());
}

cv::Matx<double, 6, 12> getCubieSpaceFromStateSpaceMatrix(const cv::Point3f& label_center)
{
    cv::Matx<double, 6, 12> cubie_space_from_state_space = camera_space_from_state_space;
    // U  R  F  D' L'  B'
    // 6  7  8  9  10  11
    if (label_center.y < -0.5)
    {
        // Affected by U
        cubie_space_from_state_space(1, 6) = 1.0;
    }
    if (label_center.x > 0.5)
    {
        // Affected by R
        cubie_space_from_state_space(0, 7) = -1.0;
    }
    if (label_center.z < -0.5)
    {
        // Affected by F
        cubie_space_from_state_space(2, 8) = 1.0;
    }
    if (label_center.y > 0.5)
    {
        // Affected by D
        cubie_space_from_state_space(1, 9) = 1.0;
    }
    if (label_center.x < -0.5)
    {
        // Affected by L
        cubie_space_from_state_space(0, 10) = -1.0;
    }
    if (label_center.z > 0.5)
    {
        // Affected by B
        cubie_space_from_state_space(2, 11) = 1.0;
    }
    return cubie_space_from_state_space;
}

std::vector<LabelObservation> generateObservationsForLabel(
    const Camera& calibrated_camera,
    const size_t label_index,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const float label_width)
{
    const float half_width = label_width * 0.5f;
    std::vector<std::vector<cv::Point3f>> candidate_points3d = [&half_width]()
    {
        std::vector<std::vector<cv::Point3f>> point_groups;
        point_groups.reserve(9);
        for (float y : {-1, 0, 1})
        {
            for (float x : {-1, 0, 1})
            {
                std::vector<cv::Point3f> label_points3d;
                label_points3d.reserve(4);
                label_points3d.emplace_back(idTo3d(0, x + half_width, y - half_width));
                label_points3d.emplace_back(idTo3d(0, x - half_width, y - half_width));
                label_points3d.emplace_back(idTo3d(0, x - half_width, y + half_width));
                label_points3d.emplace_back(idTo3d(0, x + half_width, y + half_width));
                point_groups.push_back(std::move(label_points3d));
            }
        }
        return point_groups;
    }();

    std::vector<LabelObservation> observation_candidates;
    observation_candidates.reserve(candidate_points3d.size());
    for (auto& points3d : candidate_points3d)
    {
        LabelObservation observation;
        observation.label_index = label_index;
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        cv::solvePnP(points3d, detected_corners[label_index],
            calibrated_camera.camera_matrix, calibrated_camera.dist_coeffs,
            rvec, tvec);

        // Adjust rotation vector and points to minimize rotation vector.
        adjustRotationVectorAndPoints(rvec, points3d);

        // Compute certainty of rotation and translation
        const cv::Matx66d JtJ = estimatePoseCertainty(calibrated_camera, rvec, tvec, points3d);

        const cv::Point3f label_center = pointsAverage(points3d);
        const cv::Matx<double, 6, 12> cubie_space_from_state_space =
            getCubieSpaceFromStateSpaceMatrix(label_center);

        observation.information_matrix =
            cubie_space_from_state_space.t() * JtJ * cubie_space_from_state_space;
        observation.information_vector =
            cubie_space_from_state_space.t() * JtJ * cv::Vec6d(
            rvec[0], rvec[1], rvec[2],
            tvec[0], tvec[1], tvec[2]);

        const std::vector<cv::Point2f> predicted_corners =
            projectCubeCorners(calibrated_camera, observation, label_width);
        observation.score = scorePredictedCorners(predicted_corners, detected_corners);

        observation_candidates.push_back(observation);
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

void renderPredictedCorners(
    cv::Mat3b& io_canvas,
    const std::vector<cv::Point2f>& predicted_corners,
    const std::vector<cv::Point2f>& detected_corners)
{
    for (int i = 0; i < predicted_corners.size(); i += 4)
    {
        std::vector<cv::Point> corners = {
            predicted_corners[i + 0],
            predicted_corners[i + 1],
            predicted_corners[i + 2],
            predicted_corners[i + 3],
        };
        cv::polylines(io_canvas, corners, true, cv::Scalar(0, 0, 255));
    }

    for (int i = 0; i < detected_corners.size(); i += 4)
    {
        std::vector<cv::Point> corners = {
            detected_corners[i + 0],
            detected_corners[i + 1],
            detected_corners[i + 2],
            detected_corners[i + 3],
        };
        cv::polylines(io_canvas, corners, true, cv::Scalar(255, 0, 255));
    }
}

void projectOuterCubeCornersWithUncertainties(
    const Camera& calibrated_camera,
    const LabelObservation& observation,
    std::vector<cv::Point2f>& out_points,
    std::vector<cv::Matx22f>& out_points_covariances)
{
    using PoseVector = ProbabalisticCube::PoseVector;
    using PoseMatrix = ProbabalisticCube::PoseMatrix;
    const PoseVector pose_estimate = poseEstimateFromObservation(observation);
    const cv::Mat1d observation_covariance =
        cv::Mat1d(observation.information_matrix + PoseMatrix::eye() * 1.0e-6).inv();

    const std::vector<cv::Point3f> points_in_3D = {
        cv::Point3f(-1.5, -1.5, -1.5),
        cv::Point3f(-1.5, -1.5, 1.5),
        cv::Point3f(-1.5, 1.5, -1.5),
        cv::Point3f(-1.5, 1.5, 1.5),
        cv::Point3f(1.5, -1.5, -1.5),
        cv::Point3f(1.5, -1.5, 1.5),
        cv::Point3f(1.5, 1.5, -1.5),
        cv::Point3f(1.5, 1.5, 1.5)};
    cv::Mat1d jacobian; // 2Nx(10+numDistCoeffs)
    cv::projectPoints(
        points_in_3D,
        cv::Vec3d(pose_estimate[3], pose_estimate[4], pose_estimate[5]),
        cv::Vec3d(pose_estimate[0], pose_estimate[1], pose_estimate[2]),
        calibrated_camera.camera_matrix,
        calibrated_camera.dist_coeffs,
        out_points,
        jacobian);

    out_points_covariances.resize(out_points.size());
    for (size_t i = 0; i < out_points.size(); ++i)
    {
        const cv::Matx<double, 6, 12> cubie_space_from_state_space =
            getCubieSpaceFromStateSpaceMatrix(points_in_3D[i]);
        const cv::Mat1d jacobian_extrinsics(jacobian, cv::Rect(0, 2*i, 6, 2)); // 2Nx6
        const cv::Mat1d covar_points2d =
            jacobian_extrinsics *
            cv::Mat1d(cubie_space_from_state_space) *
            observation_covariance *
            cv::Mat1d(cubie_space_from_state_space).t() *
            jacobian_extrinsics.t(); // 2Nx2N
        out_points_covariances[i] = covar_points2d;
    }
}

void renderObservationUncertainty(
    cv::Mat3b& io_canvas,
    const Camera& calibrated_camera,
    const LabelObservation& observation)
{
    std::vector<cv::Point2f> points;
    std::vector<cv::Matx22f> points_covariances;
    projectOuterCubeCornersWithUncertainties(
        calibrated_camera, observation, points, points_covariances);

    const cv::Scalar orange(0, 127, 255);
    for (auto p : points)
    {
        cv::circle(io_canvas, p, 1, orange);
    }
    std::vector<cv::Point2d> raw_ellipse_points2d;
    cv::ellipse2Poly(cv::Point2d(0,0), cv::Size2d(2,2), 0, 0, 360, 10, raw_ellipse_points2d);
    for (size_t i = 0; i < points.size(); ++i)
    {
        // Convert to double for convenience
        const cv::Matx22d point_covariance = points_covariances[i];
        const cv::Point2d point = points[i];

        cv::Vec2d eigenvalues;
        cv::Matx22d eigenvectors;
        cv::eigen(point_covariance, eigenvalues, eigenvectors);
        cv::sqrt(eigenvalues, eigenvalues);
        cv::Matx22d transform = eigenvectors * cv::Matx22d::diag(eigenvalues);
        std::vector<cv::Point> final_ellipse_points;
        for (const cv::Point2d p : raw_ellipse_points2d)
        {
            final_ellipse_points.push_back(point + transform * p);
        }
        cv::polylines(io_canvas, final_ellipse_points, true, orange);
    }
}

const LabelObservation& findBestObservation(const std::vector<LabelObservation>& observations)
{
    assert(!observations.empty());
    const LabelObservation& observation =
        *std::max_element(observations.begin(), observations.end(),
        [](const LabelObservation& a, const LabelObservation& b)
    {
        return a.score < b.score;
    });

    return observation;
}

// Encode multiple label indices based on:
// 1. A normal label_index is less than num_labels
// 2. At least one of the labels is not 0.
// The labels indices are encoded as the digits of a number with num_labels as the base.
// (instead of base 2 or base 10).
size_t packLabelIndices(size_t label_a, size_t label_b, size_t num_labels)
{
    return (label_a > label_b) ?
        label_a * num_labels + label_b :
        label_b * num_labels + label_a;
}

std::vector<size_t> unpackLabelIndices(size_t label_index, size_t num_labels)
{
    if (label_index < num_labels)
    {
        // Single index
        return {label_index};
    }
    else
    {
        // Multiple indices
        std::vector<size_t> label_indices;
        for (size_t rest = label_index; rest != 0; rest /= num_labels)
        {
            label_indices.push_back(rest % num_labels);
        }
        return label_indices;
    }
}

void renderLabelObservation(
    cv::Mat3b& io_canvas,
    const Camera& calibrated_camera,
    const LabelObservation& observation,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width)
{
    const std::vector<cv::Point2f> predicted_corners =
        projectCubeCorners(calibrated_camera, observation, label_width);

    const std::vector<size_t> label_indices =
        unpackLabelIndices(observation.label_index, detected_corners.size());

    std::vector<cv::Point2f> selected_corners;
    for (const size_t index : label_indices)
    {
        for (const auto corner : detected_corners[index])
        {
            selected_corners.push_back(corner);
        }
    }
    renderCoordinateSystem(io_canvas, calibrated_camera, observation);
    renderPredictedCorners(io_canvas, predicted_corners, selected_corners);
    renderObservationUncertainty(io_canvas, calibrated_camera, observation);
}

void showBestLabelObservation(
    const Camera& calibrated_camera,
    const std::vector<LabelObservation>& observations,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width, const cv::Mat3b& img)
{
    cv::Mat3b canvas1 = img * 0.25f;
//    cv::Mat3b canvas2 = img * 0.25f;
    if (!observations.empty())
    {
        const LabelObservation& observation = findBestObservation(observations);
        printf("detected_corners size: %lu label_index: %lu\n",
            detected_corners.size(), observation.label_index);
        renderLabelObservation(canvas1, calibrated_camera, observation, detected_corners, label_width);

//        renderLabelObservation(canvas2, calibrated_camera,
//            adjustedObservation(observation, cv::Vec3d(0, 0, 0)), detected_corners, label_width);
    }
    cv::imshow("Best label observation", canvas1);
//    cv::imshow("Adjusted label observation", canvas2);
}

double logNormPdf(
    const ProbabalisticCube::PoseVector& delta,
    const ProbabalisticCube::PoseMatrix& icov)
{
    const double tau = 2.0 * M_PI;
    return -0.5 * ((delta.t() * icov * delta)[0] + std::log(std::abs(tau / cv::determinant(icov))));
}

// FIXME(Rasmus): Update max_mahalanobis_distance!
const double max_mahalanobis_distance = 5.35; // Chi-square k=6, p=0.5
//const double max_mahalanobis_distance = 12.59; // Chi-square k=6, p=0.05

double mahalanobisDistance(
    const LabelObservation& a,
    const LabelObservation& b,
    const ProbabalisticCube::PoseMatrix& icov)
{
    return cv::Mahalanobis(
        poseEstimateFromObservation(a),
        poseEstimateFromObservation(b),
        icov);
}

ProbabalisticCube updateCube(const ProbabalisticCube& cube, const LabelObservation& observation)
{
    using PoseMatrix = ProbabalisticCube::PoseMatrix;
    using PoseVector = ProbabalisticCube::PoseVector;
    const PoseMatrix cube_pose_information_matrix = cube.pose_covariance.inv();
    const PoseVector cube_pose_information_vector = cube_pose_information_matrix * cube.pose_estimate;

    // Add information from observation to cube pose.
    PoseMatrix updated_cube_pose_information_matrix =
        cube_pose_information_matrix + observation.information_matrix;
    PoseVector updated_cube_pose_information_vector =
        cube_pose_information_vector + observation.information_vector;

    assert(std::isfinite(cube.log_likelihood));
    ProbabalisticCube updated_cube = cube;
    updated_cube.pose_covariance = updated_cube_pose_information_matrix.inv();
    updated_cube.pose_estimate = updated_cube.pose_covariance * updated_cube_pose_information_vector;

    // FIXME(Rasmus): Make proper delta considering the different coordinate systems.
    const ProbabalisticCube::PoseVector delta =
        cube.pose_estimate - poseEstimateFromObservation(observation);

    // TODO(Rasmus): Investigate values of pdfs.
    assert(std::isfinite(updated_cube.log_likelihood));
    updated_cube.log_likelihood += logNormPdf(delta, cube.pose_covariance.inv());
    assert(std::isfinite(updated_cube.log_likelihood));
    updated_cube.log_likelihood += observation.score;
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

std::vector<LabelObservation> mergeObservationsPairwise(
    const Camera& calibrated_camera,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const std::vector<LabelObservation>& observations,
    float label_width)
{
    const size_t num_labels = detected_corners.size();
    std::vector<LabelObservation> merged_observations;
    for (size_t i = 0; i < observations.size(); ++i)
    {
        for (size_t j = i + 1; j < observations.size(); ++j)
        {
            const LabelObservation& a = observations[i];
            const LabelObservation& b = observations[j];
            //const LabelObservation b = adjustedObservation(observations[j], a.rvec);
            if (a.label_index == b.label_index)
            {
                continue;
            }

            LabelObservation c;
            c.label_index = packLabelIndices(a.label_index, b.label_index, num_labels);

            c.information_matrix = a.information_matrix + b.information_matrix;
            c.information_vector = a.information_vector + b.information_vector;

            // Compute mahalanobis distance of c_vec in distribution of a and b.
            // Reject if c_vec is too unlikely.
            // Mahalanobis distance is symmetric for c_vec in a and b
            // because c_vec is the optimal combined estimate.

            const double a_mahalanobis = mahalanobisDistance(c, a, a.information_matrix);
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

    return merged_observations;
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

    const std::vector<LabelObservation> merged_observations =
        mergeObservationsPairwise(calibrated_camera, detected_corners, observations, label_width);

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
            const cv::Vec3d target_rvec(
                cube.pose_estimate[3],
                cube.pose_estimate[4],
                cube.pose_estimate[5]);
            const ProbabalisticCube updated_cube =
                updateCube(cube, observation);
            updated_cube_hypotheses.push_back(updated_cube);
        }
    }
    normalizeLikelihood(updated_cube_hypotheses);

    return updated_cube_hypotheses;
}
