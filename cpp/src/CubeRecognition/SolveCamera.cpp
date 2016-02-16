#include <cstdio>
#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Label.hpp"
#include "SolveCamera.hpp"

cv::Point3f idTo3d(int side, float id_x, float id_y)
{
    if (side == 0) // F
    {
        return cv::Point3f(id_x, -id_y, 1.5f);
    }
    if (side == 1) // R
    {
        return cv::Point3f(1.5f, -id_y, -id_x);
    }
    if (side == 2) // U
    {
        return cv::Point3f(id_x, 1.5f, id_y);
    }
    printf("unsupported side in idTo3d: %d", side);
    assert(false);
    exit(-1);
}

Camera solveCamera(
    const std::vector<std::vector<Label>>& grouped_labels,
    const std::vector<std::vector<cv::Point2f>>& spatial_indices,
    const cv::Size& image_size)
{
    assert(grouped_labels.size() >= 3);
    assert(spatial_indices.size() >= 3);

    std::vector<std::vector<cv::Point2f>> grouped_points_2d;
    std::vector<std::vector<cv::Point3f>> grouped_points_3d;
    std::vector<std::vector<cv::Point2f>> full_points_2d(1);
    std::vector<std::vector<cv::Point3f>> full_points_3d(1);
    for (size_t i = 0; i < 3; ++i)
    {
        const auto& group = grouped_labels[i];
        if (group.empty())
        {
            continue;
        }

        grouped_points_2d.push_back({});
        grouped_points_3d.push_back({});

        for (size_t j = 0; j < group.size(); ++j)
        {
            const auto& label = group[j];
            const auto& spatial_point = spatial_indices[i][j];
            grouped_points_2d.back().push_back(label.center);
            grouped_points_3d.back().emplace_back(spatial_point.x, spatial_point.y, 0.f);

            full_points_2d.back().push_back(label.center);
            full_points_3d.back().push_back(idTo3d(i, spatial_point.x, spatial_point.y));
        }
    }

    // Find initial guess of camera matrix by optimizing
    // as if there are three separate images with one checkerboard each.
    Camera cam;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    cv::calibrateCamera(grouped_points_3d, grouped_points_2d, image_size,
        cam.camera_matrix, cam.dist_coeffs, rvecs, tvecs);

    // Refine camera matrix by optimizing as a single view of a 3D cube.
    cv::calibrateCamera(full_points_3d, full_points_2d, image_size,
        cam.camera_matrix, cam.dist_coeffs, rvecs, tvecs, cv::CALIB_USE_INTRINSIC_GUESS);

    cam.rvec = rvecs[0];
    cam.tvec = tvecs[0];
    return cam;
}

std::vector<cv::Point2f> projectCube(const Camera& cam)
{
    // Generate 3D points.
    std::vector<cv::Point3f> points3d;
    for (int side = 0; side < 3; ++side)
    {
        for (int y = -1; y <= 1; ++y)
        {
            for (int x = -1; x <= 1; ++x)
            {
                points3d.push_back(idTo3d(side, x, y));
            }
        }
    }

    std::vector<cv::Point2f> points2d;
    cv::projectPoints(points3d, cam.rvec, cam.tvec, cam.camera_matrix, cam.dist_coeffs, points2d);
    return points2d;
}
