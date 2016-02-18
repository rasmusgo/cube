#include <cstdio>
#include <cstdlib>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ConnectLabels.hpp"
#include "CreateLabels.hpp"
#include "Image.hpp"
#include "Label.hpp"
#include "SolveCamera.hpp"

void setupWindows(const cv::Size& img_size)
{
    struct WinPos
    {
        std::string name;
        int x;
        int y;
    };
    std::vector<WinPos> window_positions = {
        {"detected labels", 0, 0},
        {"debug labels", img_size.width + 10, 0},
        {"F", 0, img_size.height},
        {"R", img_size.width / 3, img_size.height},
        {"U", 2 * img_size.width / 3, img_size.height},
    };
    for (auto w : window_positions)
    {
        cv::namedWindow(w.name);
        cv::moveWindow(w.name, w.x, w.y);
    }
}

void drawLabel(cv::Mat& canvas, const Label& label, const cv::Scalar& color)
{
    cv::Point2f np = label.native;

    // Find native size
    float nsize[4][4] = {
        // Type 0: 0167 x,ym
        {label.size[0], label.size[1], label.size[6], label.size[7]},
        // Type 1: 0145 x,yp
        {label.size[0], label.size[1], label.size[4], label.size[5]},
        // Type 2: 4567 yp,ym
        {label.size[4], label.size[5], label.size[6], label.size[7]},
        // Type 3 (Square): 0123 x,y
        {label.size[0], label.size[1],label.size[2], label.size[3]},
    };

    float snx = nsize[label.type][1] - nsize[label.type][0];
    float sny = nsize[label.type][3] - nsize[label.type][2];
    float snx2 = snx*0.5;
    float sny2 = sny*0.5;

    std::vector<cv::Point2f> corners = {
        label.native2xy(np.x - snx2, np.y - sny2),
        label.native2xy(np.x - snx2, np.y + sny2),
        label.native2xy(np.x + snx2, np.y + sny2),
        label.native2xy(np.x + snx2, np.y - sny2),
    };

    std::vector<cv::Point> cv_corners;
    for (auto corner : corners)
    {
        cv_corners.emplace_back(corner.x * 255, corner.y * 255);
    }
    cv::polylines(canvas, cv_corners, true, color, 1, cv::LINE_AA, 8);
}

std::vector<cv::Point2f> getLabelPositions(const cv::Mat3b img, double threshold)
{
    EdgeFunctionType edge_function = [&](const cv::Point& a, const cv::Point& b)
    {
        return cv::norm(img(a), img(b)) > threshold;
    };

    std::vector<Label> labels = createlabels(img.size(), edge_function);

    printf("Num labels: %lu\n", labels.size());
    fflush(stdout);

    cv::Mat3b canvas = img * 0.25f;
    for (auto label : labels)
    {
        drawLabel(canvas, label, cv::Scalar(255, 255, 255));
    }
    cv::imshow("detected labels", canvas);

    // Connect labels with each other in order to associate them.
    std::vector<std::vector<Label>> grouped_labels;
    std::vector<std::vector<cv::Point2f>> spatial_indices;

    std::tie(grouped_labels, spatial_indices) = connectLabels(labels);

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

std::vector<cv::Scalar> getLabelColors(const cv::Mat3b img, const std::vector<cv::Point2f>& points)
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

std::vector<cv::Point2f> getLabelPositions(const cv::Mat3b& img)
{
    std::vector<std::vector<cv::Point2f>> candidate_points(3*9);

    for (auto threshold : {4, 6, 8, 10, 12, 14, 16})
    {
        std::vector<cv::Point2f> points_top = getLabelPositions(img, threshold);

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

int main()
{
    cv::Mat3b img_top    = cv::imread("photos/IMG_6216.JPG", cv::IMREAD_COLOR);
    cv::Mat3b img_bottom = cv::imread("photos/IMG_6217.JPG", cv::IMREAD_COLOR);

    setupWindows(img_bottom.size());

    std::vector<cv::Point2f> points_top    = getLabelPositions(img_top);
    std::vector<cv::Point2f> points_bottom = getLabelPositions(img_bottom);

    std::vector<cv::Scalar> colors_top    = getLabelColors(img_top, points_top);
    std::vector<cv::Scalar> colors_bottom = getLabelColors(img_bottom, points_bottom);

    cv::Mat3b canvas_top = img_top * 0.25f;
    cv::Mat3b canvas_bottom = img_bottom * 0.25f;

    drawLabelColors(canvas_top, points_top, colors_top);
    drawLabelColors(canvas_bottom, points_bottom, colors_bottom);

    cv::imshow("top", canvas_top);
    cv::imshow("bottom", canvas_bottom);

    std::vector<cv::Scalar> label_colors;
    cv::hconcat(colors_top, colors_bottom, label_colors);

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

    cv::imshow("colors", canvas);
    cv::waitKey();
    return 0;
}
