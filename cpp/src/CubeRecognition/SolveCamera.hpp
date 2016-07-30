#pragma once

#include <tuple>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "LabelContour.hpp"

struct Camera
{
    cv::Matx33d camera_matrix;
    cv::Vec<double, 5> dist_coeffs;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
};

cv::Point3f idTo3d(int side, float id_x, float id_y);

Camera solveCamera(
    const std::vector<std::vector<LabelContour>>& grouped_labels,
    const std::vector<std::vector<cv::Point2f>>& spatial_indices,
    const cv::Size& image_size);

Camera solveCamera(
    const std::vector<cv::Point2f>& label_positions_top,
    const std::vector<cv::Point2f>& label_positions_bottom,
    const cv::Size& image_size);

std::vector<cv::Point2f> projectCube(const Camera& cam);
std::vector<cv::Point2f> projectCubeCorners(const Camera& cam, float label_width);

std::vector<Camera> predictCameraPosesForLabel(
    const Camera& cam, const std::vector<cv::Point2f>& label_corners, float label_width);

double scorePredictedCorners(const std::vector<cv::Point2f>& predicted_corners,
    const std::vector<std::vector<cv::Point2f>>& detected_corners);
