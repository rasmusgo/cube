#include <cstdio>
#include <cstdlib>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <SolverLib/Search.hpp>

#include "AssignColors.hpp"
#include "FindLabelContours.hpp"
#include "FindLabels.hpp"
#include "Image.hpp"
#include "ProbabalisticCube.hpp"
#include "SolveCamera.hpp"

const size_t kMaxNumHypotheses = 1 + 6 * 3 ; // 216
const std::vector<std::string> side_names = {"F", "R", "U", "L", "B", "D", "*"};

void setupWindows(const cv::Size& img_size)
{
    struct WinPos
    {
        std::string name;
        int x;
        int y;
    };
    std::vector<WinPos> window_positions = {
//        {"detected labels", 0, 0},
//        {"debug labels", img_size.width + 10, 0},
        {"top", 0, 0},
        {"bottom", img_size.width + 10, 0},
        {"merges", 0, img_size.height},
        {"colors", img_size.width + 25, img_size.height + 75},
        {"solution", 0, img_size.height + 180},
//        {"F", 0, img_size.height},
//        {"R", img_size.width / 3, img_size.height},
//        {"U", 2 * img_size.width / 3, img_size.height},
    };
    for (auto w : window_positions)
    {
        cv::namedWindow(w.name);
        cv::moveWindow(w.name, w.x, w.y);
    }
}

cv::Mat3b drawMoveSequence(const std::string& solution)
{
    std::stringstream ss(solution);
    std::string word;

    std::vector<std::vector<cv::Mat3b>> arrows = {
        {
            cv::imread("icons/move_F0.png"),
            cv::imread("icons/move_F1.png"),
            cv::imread("icons/move_F2.png"),
            cv::imread("icons/move_F-1.png"),
            cv::imread("icons/move_F-2.png"),
        },
        {
            cv::imread("icons/move_R0.png"),
            cv::imread("icons/move_R1.png"),
            cv::imread("icons/move_R2.png"),
            cv::imread("icons/move_R-1.png"),
            cv::imread("icons/move_R-2.png"),
        },
        {
            cv::imread("icons/move_U0.png"),
            cv::imread("icons/move_U1.png"),
            cv::imread("icons/move_U2.png"),
            cv::imread("icons/move_U-1.png"),
            cv::imread("icons/move_U-2.png"),
        },
    };

    std::vector<std::string> side_names = {"F", "R", "U"};
    std::map<std::string, cv::Mat3b> move_symbols;
    for (int side = 0; side < 3; ++side)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                std::stringstream ss;
                ss << side_names[side] << i << j;
                cv::Mat3b img;
                if (side == 0) // F
                {
                    cv::vconcat(std::vector<cv::Mat3b>{arrows[side][j], arrows[side][0], arrows[side][i]}, img);
                }
                else if (side == 1) // R
                {
                    cv::hconcat(std::vector<cv::Mat3b>{arrows[side][j], arrows[side][0], arrows[side][i]}, img);
                }
                else
                {
                    cv::vconcat(std::vector<cv::Mat3b>{arrows[side][i], arrows[side][0], arrows[side][j]}, img);
                }
                move_symbols[ss.str()] = img;
            }
        }
    }

    std::vector<cv::Mat3b> sequence_symbols;
    while (ss >> word)
    {
        if (move_symbols.count(word))
        {
            sequence_symbols.push_back(move_symbols[word]);
        }
        else
        {
            printf("Failed to find word: `%s`\n", word.c_str());
            fflush(stdout);
        }
    }

    cv::Mat3b img;
    cv::hconcat(sequence_symbols, img);
    return img;
}

std::string generateText(int i, const std::vector<size_t> &label_sides)
{
    std::stringstream ss;
    //ss << idToCubie(i);
    ss << side_names[label_sides[i]];
    return ss.str();
}

void recordVideoFrames()
{
    cv::VideoCapture cap;
    if (cap.open(0))
    {
        for(;;)
        {
            cv::Mat frame;
            cap >> frame;
            if (frame.empty())
            {
                break; // end of video stream
            }
            cv::imshow("this is you, smile! :)", frame);
            static int i = 0;
            char buf[1024];
            sprintf(buf, "frame%05d.png", i++);
            cv::imwrite(buf, frame);
            if (int key = cv::waitKey(1) & 255)
            {
                if (key == 27)
                {
                    break; // stop capturing by pressing ESC
                }
                if (key != 255)
                {
                    printf("Key: %d\n", key);
                    fflush(stdout);
                }
            }
        }
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

void showDetectedLabels(
    const cv::Mat3b& img,
    const std::vector<LabelContour>& labels,
    const std::vector<std::vector<cv::Point2f>>& detected_corners)
{
    cv::Mat3b canvas = img * 0.25f;
    drawLabels(canvas, labels, cv::Scalar(255, 255, 255));

    for (const auto& corners : detected_corners)
    {
        cv::polylines(canvas, cast<cv::Point>(corners), true, cv::Scalar(0, 0, 255));
        for (size_t i = 0; i < corners.size(); ++i)
        {
            char text[12];
            sprintf(text, "%lu", i);
            cv::putText(canvas, text, corners[i],
                cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
        }
    }

    cv::imshow("detected labels", canvas);
}

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

void printMostLikelyCubes(const std::vector<ProbabalisticCube>& cube_hypotheses)
{
    printf("Most likely cubes:\n");
    for (int i = 0; i < cube_hypotheses.size() && i < 5; ++i)
    {
        const ProbabalisticCube& cube = cube_hypotheses[i];
        const std::string permutation = cube.cube_permutation.to_String();
        const double likelihood_percent = exp(cube.log_likelihood) * 100.0;
        printf("%d: %s (%4.1f, %4.1f, %4.1f) %3.5f%%\n", i, permutation.c_str(),
            cube.pose_estimate[0], cube.pose_estimate[1], cube.pose_estimate[2],
            likelihood_percent);
    }
}

std::vector<double> scoreCameras(
    const std::vector<Camera>& all_camera_candidates,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    float label_width, const cv::Mat3b& img)
{
    std::vector<double> camera_scores;
    camera_scores.reserve(all_camera_candidates.size());
    cv::Mat1f accumulation(img.size(), 0.f);
    for (const auto& cam : all_camera_candidates)
    {
        std::vector<cv::Point2f> predicted_corners = projectCubeCorners(cam, label_width);
        double score = scorePredictedCorners(predicted_corners, detected_corners);
        camera_scores.push_back(score);

        accumulation += renderContribution(score, img.size(), predicted_corners);
    }
    double minval;
    double maxval;
    cv::minMaxLoc(accumulation, &minval, &maxval);
    cv::imshow("accumulation", accumulation / maxval);

    return camera_scores;
}

void printCameraScores(const std::vector<double>& camera_scores)
{
    printf("Camera scores:\n");
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

    // FIXME(Rasmus): change coordinate systems properly
    // camera.JtJ has order: rotation, position
    // cube pose has order: position, rotation, side rotations
    const cv::Vec6d camera_information_vector = camera.JtJ * cv::Vec6d(
        camera.rvec[0], camera.rvec[1], camera.rvec[2],
        camera.tvec[0], camera.tvec[1], camera.tvec[2]);

    // Add information from camera to cube pose.
    PoseMatrix updated_cube_pose_information_matrix = cube_pose_information_matrix;
    PoseVector updated_cube_pose_information_vector = cube_pose_information_vector;
    for (int pose_i = 0; pose_i < 6; ++pose_i)
    {
        int cam_i = (pose_i + 3) % 6;
        for (int pose_j = 0; pose_j < 6; ++pose_j)
        {
            int cam_j = (pose_j + 3) % 6;
            updated_cube_pose_information_matrix(pose_i, pose_j) += camera.JtJ(cam_i, cam_j);
        }
        updated_cube_pose_information_vector(pose_i) += camera_information_vector(cam_i);
    }

    ProbabalisticCube updated_cube = cube;
    updated_cube.pose_covariance = updated_cube_pose_information_matrix.inv();
    updated_cube.pose_estimate = updated_cube_pose_information_matrix.inv() * updated_cube_pose_information_vector;

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
        printf("No cameras predicted!");
        return cube_hypotheses;
    }

    // TODO(Rasmus): Merge camera predictions.

    const std::vector<double> camera_scores =
        scoreCameras(all_camera_candidates, detected_corners, label_width, img);

    printCameraScores(camera_scores);

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

void analyzeVideo(const std::string& folder, const Camera& calibrated_camera, float label_width)
{
    // Start with a single hypotheses of the cube.
    std::vector<ProbabalisticCube> cube_hypotheses;
    cube_hypotheses.push_back(ProbabalisticCube());
    for (int frame_i = 0;;)
    {
        printf("Frame %i\n", frame_i);

        char buf[1024];
        sprintf(buf, "%s/frame%05d.png", folder.c_str(), frame_i);
        cv::Mat3b img = cv::imread(buf, cv::IMREAD_COLOR);
        if (img.empty())
        {
            break;
        }

        {
            const size_t num_hypotheses_before = cube_hypotheses.size();
            printf("Num hypotheses before predict: %lu\n", num_hypotheses_before);

            cube_hypotheses = predict(cube_hypotheses);
            const size_t num_hypotheses_after = cube_hypotheses.size();
            printf("Num hypotheses after predict:  %lu\n", num_hypotheses_after);
            printf("Brancing factor:  %f\n", double(num_hypotheses_after) / num_hypotheses_before);

            if (num_hypotheses_after > kMaxNumHypotheses)
            {
                const size_t pruned_num = num_hypotheses_after - kMaxNumHypotheses;
                const double removed_percentage = pruned_num * 100.0 / num_hypotheses_after;
                printf("Pruning to %lu hypotheses, removing %lu (%.1f%%) hypotheses.\n",
                    kMaxNumHypotheses, pruned_num, removed_percentage);
            }
            prune(cube_hypotheses, kMaxNumHypotheses);
            printMostLikelyCubes(cube_hypotheses);
        }

        const std::vector<LabelContour> labels = findLabelContours(img, 12, true);
        const std::vector<std::vector<cv::Point2f>> detected_corners = findLabelCorners(labels);
        showDetectedLabels(img, labels, detected_corners);

        {
            const size_t num_hypotheses_before = cube_hypotheses.size();
            printf("Num hypotheses before update: %lu\n", num_hypotheses_before);

            cube_hypotheses = update(cube_hypotheses, detected_corners, calibrated_camera, label_width, img);
            const size_t num_hypotheses_after = cube_hypotheses.size();
            printf("Num hypotheses after update:  %lu\n", num_hypotheses_after);
            printf("Brancing factor:  %f\n", double(num_hypotheses_after) / num_hypotheses_before);

            if (num_hypotheses_after > kMaxNumHypotheses)
            {
                const size_t pruned_num = num_hypotheses_after - kMaxNumHypotheses;
                const double removed_percentage = pruned_num * 100.0 / num_hypotheses_after;
                printf("Pruning to %lu hypotheses, removing %lu (%.1f%%) hypotheses.\n",
                    kMaxNumHypotheses, pruned_num, removed_percentage);
            }
            prune(cube_hypotheses, kMaxNumHypotheses);
            printMostLikelyCubes(cube_hypotheses);
        }

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

void showLabelColors(
    const std::vector<size_t>& label_sides,
    const std::vector<cv::Scalar>& label_colors)
{
    cv::Mat3b canvas(cv::Size(25 * 3.1 * 6, 25 * 3), cv::Vec3b(0,0,0));

    for (int i = 0; i < 6*9; ++i)
    {
        int side = i / 9;
        int row = (i % 9) / 3;
        int col = i % 3;

        cv::Rect rect(cv::Point(25 * (side * 3.1 + col), 25 * row), cv::Size(25,25));
        cv::rectangle(canvas, rect, label_colors[i], cv::FILLED);
        cv::rectangle(canvas, rect, cv::Scalar(0,0,0), 1);
    }

    for (int i = 0; i < 6*9; ++i)
    {
        int side = i / 9;
        int row = (i % 9) / 3;
        int col = i % 3;

        cv::Point text_bl(25 * (side * 3.1 + col) + 2, 25 * row + 15);
        cv::putText(canvas, generateText(i, label_sides), text_bl,
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
    }
    cv::imshow("colors", canvas);
}

int main()
{
    //recordVideoFrames()

    cv::Mat3b img_top    = cv::imread("photos/IMG_6216.JPG", cv::IMREAD_COLOR);
    cv::Mat3b img_bottom = cv::imread("photos/IMG_6217.JPG", cv::IMREAD_COLOR);

    std::vector<cv::Point2f> points_top    = findLabelPositions(img_top);
    std::vector<cv::Point2f> points_bottom = findLabelPositions(img_bottom);

    Camera cam = solveCamera(points_top, points_bottom, img_top.size());
    const float label_width = 0.9f;
    analyzeVideo("video1", cam, label_width);

    setupWindows(img_bottom.size());

    std::vector<cv::Scalar> colors_top    = readLabelColors(img_top, points_top);
    std::vector<cv::Scalar> colors_bottom = readLabelColors(img_bottom, points_bottom);

    std::vector<cv::Scalar> label_colors;
    cv::hconcat(colors_top, colors_bottom, label_colors);
    std::vector<size_t> label_sides = assignColorsToSides(label_colors);

    cv::Mat3b canvas_top = img_top * 0.25f;
    cv::Mat3b canvas_bottom = img_bottom * 0.25f;

    std::vector<std::string> texts_top;
    for (int i = 0; i < 3*9; ++i)
    {
        texts_top.push_back(generateText(i, label_sides));
    }
    std::vector<std::string> texts_bottom;
    for (int i = 3*9; i < 6*9; ++i)
    {
        texts_bottom.push_back(generateText(i, label_sides));
    }

    drawLabelInfo(canvas_top, points_top, texts_top, cv::Scalar(255, 255, 255), 1.0);
    drawLabelInfo(canvas_bottom, points_bottom, texts_bottom, cv::Scalar(255, 255, 255), 1.0);

    cv::imshow("top", canvas_top);
    cv::imshow("bottom", canvas_bottom);

    showLabelColors(label_sides, label_colors);

    const std::vector<size_t> kociemba_order = {
        18, 19, 20, 21, 22, 23, 24, 25, 26, // U
         9, 10, 11, 12, 13, 14, 15, 16, 17, // R
         0,  1,  2,  3,  4,  5,  6,  7,  8, // F
        51, 48, 45, 52, 49, 46, 53, 50, 47, // D
        35, 34, 33, 32, 31, 30, 29, 28, 27, // L
        44, 43, 42, 41, 40, 39, 38, 37, 36, // B
    };
    std::stringstream ss;
    for (size_t i : kociemba_order)
    {
        ss << side_names[label_sides[i]];
    }
    printf("Cube state: %s\n", ss.str().c_str());
    fflush(stdout);

    twophase::Search search;
    std::string solution = search.solution(ss.str(), 18, 15, false);
    printf("Solution: %s\n", solution.c_str());
    fflush(stdout);

    cv::Mat3b solution_img = drawMoveSequence(solution);
    cv::imshow("solution", solution_img);

    cv::waitKey();
    return 0;
}
