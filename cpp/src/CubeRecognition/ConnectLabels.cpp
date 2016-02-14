#include <cstdio>
#include <sstream>

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

        for (const auto& label : group)
        {
            float rel_x = (label.native.x - min_x) / (max_x - min_x);
            float rel_y = (label.native.y - min_y) / (max_y - min_y);
            int id_width  = std::round(3 * label.native_rect.width  / (max_x - min_x));
            int id_height = std::round(3 * label.native_rect.height / (max_y - min_y));
            float id_x = (id_width  == 2 ? std::round(rel_x * 3) - 0.5f : std::round(rel_x * 3 - 0.5f));
            float id_y = (id_height == 2 ? std::round(rel_y * 3) - 0.5f : std::round(rel_y * 3 - 0.5f));

            std::stringstream text;
            text << id_x << "," << id_y;
            cv::Point text_position(label.native.x - min_x, label.native.y - min_y);
            int baseline;
            cv::Size text_size = cv::getTextSize(text.str(), cv::FONT_HERSHEY_PLAIN, 1.0, 1, &baseline);
            text_position.x -= text_size.width / 2;
            text_position.y += text_size.height / 2;
            cv::putText(canvas, text.str(), text_position, cv::FONT_HERSHEY_PLAIN, 1.0, colors[i]);
        }

        cv::imshow(winnames[i], canvas);
    }
}
