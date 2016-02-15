#include <cstdio>
#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ConnectLabels.hpp"

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

void solveCamera(
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
    cv::Matx33d camera_matrix;
    cv::Vec<double, 5> dist_coeffs;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    cv::calibrateCamera(grouped_points_3d, grouped_points_2d, image_size,
        camera_matrix, dist_coeffs, rvecs, tvecs);

    std::cout << "camera_matrix:\n" << camera_matrix << std::endl;

    // Refine camera matrix by optimizing as a single view of a 3D cube.
    cv::calibrateCamera(full_points_3d, full_points_2d, image_size,
        camera_matrix, dist_coeffs, rvecs, tvecs, cv::CALIB_USE_INTRINSIC_GUESS);

    std::cout << "camera_matrix:\n" << camera_matrix << std::endl;
}

/**
 * Connect labels by associating neighbors in a connected components search.
 *
 * @return (grouped_labels, spatial_indices)
 */
std::pair<std::vector<std::vector<Label>>, std::vector<std::vector<cv::Point2f>>>
    connectLabels(const std::vector<Label>& labels)
{
    std::vector<std::vector<Label>> typed_labels(4);
    for (const auto& label : labels)
    {
        typed_labels[label.type].push_back(label);
    }

    cv::Scalar colors[] = {
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 0, 255),
        cv::Scalar(0, 255, 255),
        cv::Scalar(255, 255, 255),
    };
    std::string winnames[] = {"F", "R", "U", "S"};

    std::vector<std::vector<cv::Point2f>> spatial_indices(4);
    for (size_t i = 0; i < 4; ++i)
    {
        const auto& group = typed_labels[i];

        if (group.empty())
            continue;

        float min_x = 99999999;
        float max_x = -99999999;
        float min_y = 99999999;
        float max_y = -99999999;
        for (const auto& label : group)
        {
            min_x = std::min(min_x, label.native_rect.x);
            max_x = std::max(max_x, label.native_rect.x + label.native_rect.width);
            min_y = std::min(min_y, label.native_rect.y);
            max_y = std::max(max_y, label.native_rect.y + label.native_rect.height);
        }
        cv::Mat3b canvas(cv::Size(max_x - min_x, max_y - min_y), cv::Vec3b(0,0,0));
        for (const auto& label : group)
        {
            cv::Rect rect = label.native_rect - cv::Point2f(min_x, min_y);
            cv::rectangle(canvas, rect, colors[i]);
        }

        for (const auto& label : group)
        {
            float rel_x = (label.native.x - min_x) / (max_x - min_x);
            float rel_y = (label.native.y - min_y) / (max_y - min_y);
            int id_width  = std::round(3 * label.native_rect.width  / (max_x - min_x));
            int id_height = std::round(3 * label.native_rect.height / (max_y - min_y));
            float id_x = (id_width  == 2 ? std::round(rel_x * 3) - 1.5f : std::round(rel_x * 3 - 1.5f));
            float id_y = (id_height == 2 ? std::round(rel_y * 3) - 1.5f : std::round(rel_y * 3 - 1.5f));

            spatial_indices[i].emplace_back(id_x, id_y);

            std::stringstream text;
            text << id_x << "," << id_y;
            cv::Point text_position(label.native.x - min_x, label.native.y - min_y);
            cv::Size text_size = cv::getTextSize(text.str(), cv::FONT_HERSHEY_PLAIN, 1.0, 1, NULL);
            text_position.x -= text_size.width / 2;
            text_position.y += text_size.height / 2;
            cv::putText(canvas, text.str(), text_position, cv::FONT_HERSHEY_PLAIN, 1.0, colors[i]);
        }

        cv::imshow(winnames[i], canvas);
    }

    return {typed_labels, spatial_indices};
}
