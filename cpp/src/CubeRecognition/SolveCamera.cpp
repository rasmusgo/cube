#include <cstdio>
#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "LabelContour.hpp"
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
    if (side == 3) // B
    {
        return cv::Point3f(-id_x, -id_y, -1.5f);
    }
    if (side == 4) // L
    {
        return cv::Point3f(-1.5f, -id_y, id_x);
    }
    if (side == 5) // D
    {
        return cv::Point3f(-id_x, -1.5f, id_y);
    }
    printf("unsupported side in idTo3d: %d", side);
    assert(false);
    exit(-1);
}

Camera solveCamera(
    const std::vector<std::vector<LabelContour>>& grouped_labels,
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

Camera solveCamera(
    const std::vector<cv::Point2f>& label_positions_top,
    const std::vector<cv::Point2f>& label_positions_bottom,
    const cv::Size& image_size)
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

    std::vector<std::vector<cv::Point2f>> grouped_points_2d(6);
    std::vector<std::vector<cv::Point3f>> grouped_points_3d(6);
    auto it_top    = label_positions_top.begin();
    auto it_bottom = label_positions_bottom.begin();
    for (size_t i = 0; i < 3; ++i)
    {
        for (int y = -1; y <= 1; ++y)
        {
            for (int x = -1; x <= 1; ++x)
            {
                grouped_points_2d[i].push_back(*it_top);
                grouped_points_3d[i].emplace_back(x, y, 0.f);
                ++it_top;
                grouped_points_2d[i + 3].push_back(*it_bottom);
                grouped_points_3d[i + 3].emplace_back(x, y, 0.f);
                ++it_bottom;
            }
        }
    }

    // Find initial guess of camera matrix by optimizing
    // as if there are six separate images with one checkerboard each.
    Camera cam;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    cv::calibrateCamera(grouped_points_3d, grouped_points_2d, image_size,
        cam.camera_matrix, cam.dist_coeffs, rvecs, tvecs);

    // Refine camera matrix by optimizing as two views of a 3D cube.
    std::vector<std::vector<cv::Point2f>> full_points_2d =
        { label_positions_top, label_positions_bottom };
    std::vector<std::vector<cv::Point3f>> full_points_3d =
        { points3d, points3d };
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

std::vector<cv::Point2f> projectCubeCorners(const Camera& cam, float label_width)
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
                points3d.push_back(idTo3d(side, x + half_width, y - half_width));
                points3d.push_back(idTo3d(side, x - half_width, y - half_width));
                points3d.push_back(idTo3d(side, x - half_width, y + half_width));
                points3d.push_back(idTo3d(side, x + half_width, y + half_width));
            }
        }
    }

    std::vector<cv::Point2f> points2d;
    cv::projectPoints(points3d, cam.rvec, cam.tvec, cam.camera_matrix, cam.dist_coeffs, points2d);

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
        cam_candidates.push_back(std::move(new_cam));
    }
    return cam_candidates;
}
