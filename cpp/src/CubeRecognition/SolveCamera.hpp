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

void renderCoordinateSystem(cv::Mat3b& io_canvas, const Camera& cam);
