#include <cstdio>
#include <numeric>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <SolverLib/Search.hpp>

#include "FindLabelContours.hpp"
#include "FindLabels.hpp"
#include "Image.hpp"
#include "ObserveLabels.hpp"
#include "SolveCamera.hpp"
#include "Timer.hpp"

void handleKeys(int& io_frame_i, int key)
{
    if (key == 32) // space
    {
        ++io_frame_i;
    }
    if (key == 83 || key == 3) // right arrow
    {
        ++io_frame_i;
    }
    if (key == 82) // up arrow
    {

    }
    if (key == 81 || key == 2) // left arrow
    {
        --io_frame_i;
    }
    if (key == 84 || key == 1) // down arrow
    {

    }
    if (key != 255)
    {
        printf("Key: %d\n", key);
        fflush(stdout);
    }
}

void sortPointsWithCovariances(
    std::vector<cv::Point2f>& io_points,
    std::vector<cv::Matx22f>& io_points_covariances)
{
    std::vector<size_t> ordering(io_points.size());
    std::iota(ordering.begin(), ordering.end(), 0); // Fill with identity permutation
    std::sort(ordering.begin(), ordering.end(),
        [&io_points](size_t a, size_t b)
    {
        return io_points[a].y < io_points[b].y; // Sort by y-coordinate
    });
    std::vector<cv::Point2f> ordered_points;
    std::vector<cv::Matx22f> ordered_points_covariances;
    for (size_t index : ordering)
    {
        ordered_points.push_back(io_points[index]);
        ordered_points_covariances.push_back(io_points_covariances[index]);
    }
    io_points = std::move(ordered_points);
    io_points_covariances = std::move(ordered_points_covariances);
}

void analyzeVideo(const std::string& folder, const Camera& calibrated_camera, float label_width)
{
    for (int frame_i = 0;;)
    {
        printf("Frame %i\n", frame_i);
        startTimer();

        char buf[1024];
        sprintf(buf, "%s/frame%05d.png", folder.c_str(), frame_i);
        cv::Mat3b img = cv::imread(buf, cv::IMREAD_COLOR);
        if (img.empty())
        {
            break;
        }

        const std::vector<LabelContour> labels = findLabelContours(img, 12, true);
        const std::vector<std::vector<cv::Point2f>> detected_corners = findLabelCorners(labels);
        showDetectedLabels(img, labels, detected_corners);
        printf("Found %lu label candidates.\n", labels.size());

        const std::vector<LabelObservation> observations =
            generateObservations(calibrated_camera, detected_corners, label_width);

        cv::Mat3b canvas = img * 0.25f;
        if (!observations.empty())
        {
            const LabelObservation observation = findBestObservation(observations);
            renderLabelObservation(canvas, calibrated_camera, observation, detected_corners, label_width);

            std::vector<cv::Point2f> original_points;
            std::vector<cv::Matx22f> original_points_covariances;
            projectOuterCubeCornersWithUncertainties(calibrated_camera, observation,
                original_points, original_points_covariances);
            sortPointsWithCovariances(original_points, original_points_covariances);

            const std::vector<cv::Vec3d> target_vectors = {
                cv::Vec3d(0, 0, 0),
                cv::Vec3d(M_PI_2, 0, 0),
                cv::Vec3d(0, M_PI_2, 0),
                cv::Vec3d(0, 0, M_PI_2),
                cv::Vec3d(-M_PI_2, 0, 0),
                cv::Vec3d(0, -M_PI_2, 0),
                cv::Vec3d(0, 0, -M_PI_2),
                cv::Vec3d(M_PI, 0, 0),
                cv::Vec3d(0, M_PI, 0),
                cv::Vec3d(0, 0, M_PI),
            };
            double sum_squared_errors = 0;
            for (const cv::Vec3d target_vec : target_vectors)
            {
                const LabelObservation adjusted_observation =
                    adjustedObservation(observation, target_vec);
                renderLabelObservation(canvas, calibrated_camera,
                    adjusted_observation, detected_corners, label_width);

                std::vector<cv::Point2f> adjusted_points;
                std::vector<cv::Matx22f> adjusted_points_covariances;
                projectOuterCubeCornersWithUncertainties(calibrated_camera, adjusted_observation,
                    adjusted_points, adjusted_points_covariances);
                sortPointsWithCovariances(adjusted_points, adjusted_points_covariances);

                // Compare points!
                assert(adjusted_points.size() == original_points.size());
                for (size_t i = 0; i < adjusted_points.size(); ++i)
                {
                    assert(cv::norm(cv::Mat(adjusted_points[i]),
                        cv::Mat(original_points[i]), cv::NORM_L2SQR) < 1.0);
                    sum_squared_errors += cv::norm(cv::Mat(adjusted_points_covariances[i]),
                        cv::Mat(original_points_covariances[i]), cv::NORM_L2SQR);
                }
            }
            const double rmse = std::sqrt(sum_squared_errors /
                double(original_points.size() * target_vectors.size()));
            printf("Adjusted points covariances root of mean squared error: %f\n", rmse);
        }
        cv::imshow("Superimposed adjustments", canvas);

        //showBestLabelObservation(calibrated_camera, observations, detected_corners, label_width, img);

        stopTimer();
        fflush(stdout);

        if (int key = cv::waitKey(0) & 255)
        {
            if (key == 27)
            {
                break; // stop by pressing ESC
            }
            handleKeys(frame_i, key);
        }
    }
}

int main()
{
    cv::Mat3b img_top    = cv::imread("photos/IMG_6216.JPG", cv::IMREAD_COLOR);
    cv::Mat3b img_bottom = cv::imread("photos/IMG_6217.JPG", cv::IMREAD_COLOR);

    std::vector<cv::Point2f> points_top    = findLabelPositions(img_top);
    std::vector<cv::Point2f> points_bottom = findLabelPositions(img_bottom);

    const Camera calibrated_camera = solveCamera(points_top, points_bottom, img_top.size());
    const float label_width = 0.9f;
    analyzeVideo("video3", calibrated_camera, label_width);

    return 0;
}
