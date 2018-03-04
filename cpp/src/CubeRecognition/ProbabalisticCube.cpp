#include "ProbabalisticCube.hpp"

#include <cstdio>
#include <cstdlib>

#include <opencv2/opencv.hpp>

#include <SolverLib/FaceCube.hpp>

#include "SolveCamera.hpp"

const cv::Matx33d urf_from_xyz(
    0, 1, 0,
    -1, 0, 0,
    0, 0, 1);

static const double angle_increment = 22.5 / 180 * M_PI;
static const ProbabalisticCube::PoseMatrix pose_prediction_uncertainty =
    ProbabalisticCube::PoseMatrix::diag(ProbabalisticCube::PoseMatrix::diag_type(
    1.0, 1.0, 1.0, // position
    angle_increment, angle_increment, angle_increment, // rotation
    angle_increment, angle_increment, angle_increment, // side rotations (URF)
    angle_increment, angle_increment, angle_increment)); // side rotations (D'L'B')

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

cv::Vec3d composeRotation(const cv::Vec3d& a, const cv::Vec3d& b)
{
    cv::Matx33d a_mat;
    cv::Rodrigues(a, a_mat);
    cv::Matx33d b_mat;
    cv::Rodrigues(b, b_mat);
    cv::Vec3d c;
    cv::Rodrigues(a_mat * b_mat, c);
    return c;
}

void propagateContinousRotationsToDiscrete(ProbabalisticCube& io_cube)
{
    const double turning_threshold = M_PI / 3.0; // Two thirds of a "move"
    // TODO(Rasmus): Whole cube rotations!
    cv::Vec3d rvec(io_cube.pose_estimate[3], io_cube.pose_estimate[4], io_cube.pose_estimate[5]);

    bool has_moved_whole_cube = false;
    while (
        std::abs(rvec[0]) > turning_threshold ||
        std::abs(rvec[1]) > turning_threshold ||
        std::abs(rvec[2]) > turning_threshold)
    {
        double max_rotation_i = 0;
        double max_rotation_magnitude = 0;
        for (int i = 0; i < 3; ++i)
        {
            const double rotation_magnitude = std::abs(rvec[i]);
            if (rotation_magnitude > max_rotation_magnitude)
            {
                max_rotation_i = i;
                max_rotation_magnitude = rotation_magnitude;
            }
        }
        cv::Vec3d delta_rvec(0, 0, 0);
        delta_rvec[max_rotation_i] = rvec[max_rotation_i] > 0 ? M_PI_2 : -M_PI_2;

        // Subtract from continuous part.
        rvec = composeRotation(rvec, -delta_rvec);

        // TODO(Rasmus): Update covariance matrix to handle side rotations (face moves) properly.

        // Add to discrete part.
        cv::Vec3d delta_urf = urf_from_xyz * delta_rvec / M_PI_2;
        for (int i = 0; i < 3; ++i)
        {
            const int direction = std::round(delta_urf[i]);
            if (direction != 0)
            {
                io_cube.cube_permutation.moveAxis(i, direction, direction, direction);
            }
        }
        if (has_moved_whole_cube)
        {
            printf("WARNING: Executing multiple whole cube moves in propagateContinousRotationsToDiscrete!\n");
        }
        has_moved_whole_cube = true;
    }

    // Handle face rotations
    bool has_moved_face = false;
    for (int i = 0; i < 6; ++i)
    {
        assert(std::isfinite(io_cube.pose_estimate[i + 6]));
        while (std::abs(io_cube.pose_estimate[i + 6]) > turning_threshold)
        {
            const int direction = io_cube.pose_estimate[i + 6] > 0 ? 1 : -1;
            io_cube.pose_estimate[i + 6] -= direction * M_PI_2;
            io_cube.cube_permutation.moveAxis(
                i % 3,
                i < 3 ? direction : 0, // U,R,F
                0,
                i >= 3 ? direction : 0); // D',L',B'
            if (has_moved_face)
            {
                printf("WARNING: Executing multiple face moves in propagateContinousRotationsToDiscrete!\n");
            }
            has_moved_face = true;
        }
    }
}

std::vector<ProbabalisticCube> generatePredictions(const ProbabalisticCube& parent)
{
    assert(std::isfinite(parent.log_likelihood));
    // Create a new cube pose distribution by applying possible moves
    // and rating the likelihood of them occuring.
    // The moves considered are:
    // * Whole cube rotations (+/-22.5 degrees, 3 axis = 6 moves)
    // * Face moves (6 faces, +/- 22.5 degrees = 12 moves)

    ProbabalisticCube child_template = parent;
    child_template.pose_covariance += pose_prediction_uncertainty;
    child_template.log_likelihood = std::log(1.0); // One "part" likelihood

    std::vector<ProbabalisticCube> children;
    for (int i = 0; i < 3; ++i)
    {
        for (int direction : {-1, 1})
        {
            { // Whole cube rotation
                ProbabalisticCube child = child_template;
                child.pose_estimate[i + 3] += angle_increment * direction;
                children.push_back(std::move(child));
            }
            { // Nearby side rotation
                ProbabalisticCube child = child_template;
                child.pose_estimate[i + 6] += angle_increment * direction;
                children.push_back(std::move(child));
            }
            { // Remote side rotation
                ProbabalisticCube child = child_template;
                child.pose_estimate[i + 9] += angle_increment * direction;
                children.push_back(std::move(child));
            }
        }
    }
    // No rotation
    children.push_back(child_template);
    // Likelihood of not moving any side is considered 50% (as likely as all moves combined).
    children.back().log_likelihood = std::log(children.size() - 1);

    // Propagate rotations from continuous part of representation to discrete part.
    for (ProbabalisticCube& child : children)
    {
        propagateContinousRotationsToDiscrete(child);
    }

    // Normalize and multiply with parent likelihood
    normalizeLikelihood(children);
    for (ProbabalisticCube& child : children)
    {
        child.log_likelihood += parent.log_likelihood; // Multiply with parent likelihood.
        assert(std::isfinite(child.log_likelihood));
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
                const double max_difference = 0.1;
                if (!closeToEqual(cube.pose_estimate, it->second.pose_estimate, max_difference))
                {
                    continue;
                }
                return it;
            }
            return equal_range.second;
        }();

        if (match != equal_range.second)
        {
            // We found a match!
            const ProbabalisticCube& a = match->second;
            const ProbabalisticCube& b = cube;
            double max_log_likelihood = std::max(a.log_likelihood, b.log_likelihood);
            match->second.log_likelihood = max_log_likelihood + log(
                exp(a.log_likelihood - max_log_likelihood) +
                exp(b.log_likelihood - max_log_likelihood));
            // Update covariance by covariance intersection.
            // Minimize trace of covariance matrix by interpolation of information matrices.
            const ProbabalisticCube::PoseMatrix a_information_matrix = a.pose_covariance.inv();
            const ProbabalisticCube::PoseMatrix b_information_matrix = b.pose_covariance.inv();
            double best_t = 1.0;
            double best_trace = cv::trace(b.pose_covariance);
            const size_t num_tests = 10;
            for (size_t i = 0; i < num_tests; ++i)
            {
                const double t = double(i) / double(num_tests);
                const ProbabalisticCube::PoseMatrix interpolated_information_matrix =
                    a_information_matrix * (1.0 - t) + b_information_matrix * t;
                const double trace = cv::trace(interpolated_information_matrix.inv());
                if (trace < best_trace)
                {
                    best_t = t;
                    best_trace = trace;
                }
            }
            const ProbabalisticCube::PoseMatrix interpolated_information_matrix =
                a_information_matrix * (1.0 - best_t) + b_information_matrix * best_t;
            match->second.pose_covariance = interpolated_information_matrix.inv();

            const ProbabalisticCube::PoseVector a_information_vector = a_information_matrix * a.pose_estimate;
            const ProbabalisticCube::PoseVector b_information_vector = b_information_matrix * b.pose_estimate;
            const ProbabalisticCube::PoseVector interpolated_information_vector =
                a_information_vector * (1.0 - best_t) + b_information_vector * best_t;
            match->second.pose_estimate =
                match->second.pose_covariance * interpolated_information_vector;
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

std::vector<cv::Point2f> pickVisibleLabels(const std::vector<cv::Point2f>& points2d)
{
    std::vector<cv::Point2f> visible_points2d;
    for (int i = 0; i < points2d.size(); i += 4)
    {
        // Pick 4 points that make up a label.
        cv::Mat1f contour = cv::Mat(points2d).reshape(1, points2d.size())(cv::Rect(0, i, 2, 4));
        // Backface or frontface?
        if (cv::contourArea(contour, true) < 0)
        {
            for (int j = 0; j < 4; ++j)
            {
                visible_points2d.push_back(points2d[i + j]);
            }
        }
    }

    return visible_points2d;
}

std::vector<cv::Point2f> projectCubeCorners(
    const Camera& calibrate_camera,
    const ProbabalisticCube& cube,
    float label_width)
{
    // Generate 3D points.
    const float half_width = label_width * 0.5f;
    std::vector<cv::Point3f> points3d;
    for (int side = 0; side < 6; ++side)
    {
        for (int y : {-1, 0, 1})
        {
            for (int x : {-1, 0, 1})
            {
                const cv::Point3f label_center = idTo3d(side, x, y);
                cv::Vec3f cubie_rvec(0, 0, 0);
                // U  R  F  D' L'  B'
                // 6  7  8  9  10  11
                if (label_center.y < 0)
                {
                    // Affected by U
                    cubie_rvec[1] += cube.pose_estimate[6];
                }
                if (label_center.x > 0)
                {
                    // Affected by R
                    cubie_rvec[0] -= cube.pose_estimate[7];
                }
                if (label_center.z < 0)
                {
                    // Affected by F
                    cubie_rvec[2] += cube.pose_estimate[8];
                }
                if (label_center.y > 0)
                {
                    // Affected by D
                    cubie_rvec[1] += cube.pose_estimate[9];
                }
                if (label_center.x < 0)
                {
                    // Affected by L
                    cubie_rvec[0] -= cube.pose_estimate[10];
                }
                if (label_center.z > 0)
                {
                    // Affected by B
                    cubie_rvec[2] += cube.pose_estimate[11];
                }
                cv::Matx33f cubie_rmat;
                cv::Rodrigues(cubie_rvec, cubie_rmat);
                points3d.push_back(cubie_rmat * idTo3d(side, x + half_width, y - half_width));
                points3d.push_back(cubie_rmat * idTo3d(side, x - half_width, y - half_width));
                points3d.push_back(cubie_rmat * idTo3d(side, x - half_width, y + half_width));
                points3d.push_back(cubie_rmat * idTo3d(side, x + half_width, y + half_width));
            }
        }
    }

    const cv::Vec3d rvec(cube.pose_estimate[3], cube.pose_estimate[4], cube.pose_estimate[5]);
    const cv::Vec3d tvec(cube.pose_estimate[0], cube.pose_estimate[1], cube.pose_estimate[2]);
    std::vector<cv::Point2f> points2d;
    cv::projectPoints(points3d, rvec, tvec,
        calibrate_camera.camera_matrix,
        calibrate_camera.dist_coeffs,
        points2d);

    return pickVisibleLabels(points2d);
}

void renderCoordinateSystem(
    cv::Mat3b& io_canvas,
    const Camera& calibrated_camera,
    const ProbabalisticCube& cube)
{
    Camera cam = calibrated_camera;
    cam.rvec = cv::Vec3d(cube.pose_estimate[3], cube.pose_estimate[4], cube.pose_estimate[5]);
    cam.tvec = cv::Vec3d(cube.pose_estimate[0], cube.pose_estimate[1], cube.pose_estimate[2]);
    renderCoordinateSystem(io_canvas, cam);
}
