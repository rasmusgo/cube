#include <cstdio>
#include <cstdlib>
#include <functional>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ConnectLabelContours.hpp"
#include "FindLabelContours.hpp"
#include "FindLabels.hpp"
#include "LabelContour.hpp"
#include "SolveCamera.hpp"

std::vector<cv::Point2f> findLabelPositions(const cv::Mat3b& img)
{
    std::vector<std::vector<cv::Point2f>> candidate_points(3*9);

    for (auto threshold : {4, 6, 8, 10, 12, 14, 16})
    {
        std::vector<cv::Point2f> points_top = findLabelPositions(img, threshold);

        if (!points_top.empty())
        {
            for (int i = 0; i < 3*9; ++i)
            {
                candidate_points[i].push_back(points_top[i]);
            }
        }
    }

    std::vector<cv::Point2f> median_points;

    for (const auto& points : candidate_points)
    {
        assert(!points.empty());

        std::vector<float> px;
        std::vector<double> py;
        for (const auto& point : points)
        {
            px.push_back(point.x);
            py.push_back(point.y);
        }

        size_t mid = points.size() / 2;
        std::nth_element(px.begin(), px.begin() + mid, px.end());
        std::nth_element(py.begin(), py.begin() + mid, py.end());

        median_points.emplace_back(px[mid], py[mid]);
    }

    return median_points;
}

std::vector<cv::Point2f> findLabelPositions(const cv::Mat3b& img, double threshold)
{
    EdgeFunctionType edge_function = [&](const cv::Point& a, const cv::Point& b)
    {
        return cv::norm(img(a), img(b)) > threshold;
    };

    std::vector<LabelContour> labels = findLabelContours(img.size(), edge_function);

    printf("Num labels: %lu\n", labels.size());
    fflush(stdout);

    cv::Mat3b canvas = img * 0.25f;
    drawLabels(canvas, labels, cv::Scalar(255, 255, 255));
    cv::imshow("detected labels", canvas);

    // Connect labels with each other in order to associate them.
    std::vector<std::vector<LabelContour>> grouped_labels;
    std::vector<std::vector<cv::Point2f>> spatial_indices;

    std::tie(grouped_labels, spatial_indices) = connectLabelContours(labels);

    Camera cam;
    try
    {
        cam = solveCamera(grouped_labels, spatial_indices, img.size());
    }
    catch (cv::Exception)
    {
        return {};
    }

    return projectCube(cam);
}

std::vector<cv::Scalar> readLabelColors(const cv::Mat3b img, const std::vector<cv::Point2f>& points)
{
    std::vector<cv::Scalar> label_colors;
    cv::Rect img_rect(cv::Point(0,0), img.size());
    for (const auto& p :points)
    {
        if (img_rect.contains(p))
        {
            label_colors.push_back(img(p));
        }
    }

    if (label_colors.size() == 9*3)
    {
        return label_colors;
    }
    else
    {
        return {};
    }
}

void drawLabelColors(cv::Mat3b& canvas,
    const std::vector<cv::Point2f>& points, const std::vector<cv::Scalar>& colors)
{
    for (size_t i = 0; i < points.size(); ++i)
    {
        cv::circle(canvas, points[i], 5, colors[i], cv::FILLED);
        cv::circle(canvas, points[i], 5, cv::Scalar(255, 255, 255));
    }
}

void drawLabelInfo(cv::Mat3b& canvas,
    const std::vector<cv::Point2f>& points,
    const std::vector<std::string>& texts,
    const cv::Scalar color,
    double font_scale)
{
    for (size_t i = 0; i < points.size(); ++i)
    {
        cv::Size text_size = cv::getTextSize(texts[i], cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, NULL);
        cv::Point text_pos(points[i].x - text_size.width / 2, points[i].y + text_size.height / 2);
        cv::putText(canvas, texts[i], text_pos, cv::FONT_HERSHEY_SIMPLEX, font_scale, color);
    }
}
