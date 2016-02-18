#include <cstdio>
#include <cstdlib>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "AssignColors.hpp"
#include "FindLabels.hpp"
#include "Image.hpp"

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
        {"top", 0, 0},
        {"bottom", img_size.width + 10, 0},
        {"merges", 0, img_size.height},
        {"colors", img_size.width + 25, img_size.height + 75},
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

int main()
{
    cv::Mat3b img_top    = cv::imread("photos/IMG_6216.JPG", cv::IMREAD_COLOR);
    cv::Mat3b img_bottom = cv::imread("photos/IMG_6217.JPG", cv::IMREAD_COLOR);

    setupWindows(img_bottom.size());

    std::vector<cv::Point2f> points_top    = findLabelPositions(img_top);
    std::vector<cv::Point2f> points_bottom = findLabelPositions(img_bottom);

    std::vector<cv::Scalar> colors_top    = readLabelColors(img_top, points_top);
    std::vector<cv::Scalar> colors_bottom = readLabelColors(img_bottom, points_bottom);

    std::vector<cv::Scalar> label_colors;
    cv::hconcat(colors_top, colors_bottom, label_colors);
    std::vector<size_t> label_sides = assignColorsToSides(label_colors);

    cv::Mat3b canvas_top = img_top * 0.25f;
    cv::Mat3b canvas_bottom = img_bottom * 0.25f;

    const std::vector<std::string> side_names = {"F", "R", "U", "L", "B", "D", "*"};

    auto generate_text = [&](int i)
    {
        std::stringstream ss;
        //ss << idToCubie(i);
        ss << side_names[label_sides[i]];
        return ss.str();
    };

    std::vector<std::string> texts_top;
    for (int i = 0; i < 3*9; ++i)
    {
        texts_top.push_back(generate_text(i));
    }
    std::vector<std::string> texts_bottom;
    for (int i = 3*9; i < 6*9; ++i)
    {
        texts_bottom.push_back(generate_text(i));
    }

    drawLabelInfo(canvas_top, points_top, texts_top, cv::Scalar(255, 255, 255), 1.0);
    drawLabelInfo(canvas_bottom, points_bottom, texts_bottom, cv::Scalar(255, 255, 255), 1.0);

    cv::imshow("top", canvas_top);
    cv::imshow("bottom", canvas_bottom);

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
        cv::putText(canvas, generate_text(i), text_bl,
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
    }

    cv::imshow("colors", canvas);
    cv::waitKey();
    return 0;
}
