#include <cstdio>
#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ConnectLabelContours.hpp"

/**
 * Connect labels by associating neighbors in a connected components search.
 *
 * @return (grouped_labels, spatial_indices)
 */
std::pair<std::vector<std::vector<LabelContour>>, std::vector<std::vector<cv::Point2f>>>
    connectLabelContours(const std::vector<LabelContour>& labels)
{
    std::vector<std::vector<LabelContour>> typed_labels(4);
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
