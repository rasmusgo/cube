#include <cstdio>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ConnectLabels.hpp"

/**
 * Connect labels by associating neighbors in a connected components search.
 */
void connectLabels(const std::vector<Label>& labels)
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

    int i = -1;
    for (const auto& group : typed_labels)
    {
        ++i;
        if (group.empty())
            continue;

        float min_x = 99999999;
        float max_x = -99999999;
        float min_y = 99999999;
        float max_y = -99999999;
        for (const auto& label : group)
        {
            min_x = std::min(min_x, label.native_rect.x);
            max_x = std::max(max_x, label.native_rect.br().x);
            min_y = std::min(min_y, label.native_rect.y);
            max_y = std::max(max_y, label.native_rect.br().y);
        }
        cv::Mat3b canvas(cv::Size(max_x - min_x, max_y - min_y), cv::Vec3b(0,0,0));
        for (const auto& label : group)
        {
            cv::Rect rect = label.native_rect - cv::Point2f(min_x, min_y);
            cv::rectangle(canvas, rect, colors[i]);
        }
        cv::imshow(winnames[i], canvas);
    }
}
