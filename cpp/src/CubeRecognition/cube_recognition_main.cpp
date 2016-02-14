#include <cstdio>
#include <cstdlib>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Image.hpp"

#include "Label.hpp"
#include "CreateLabels.hpp"

int main()
{
    cv::Mat3b img = cv::imread("cube_0b.png", cv::IMREAD_COLOR);

    EdgeFunctionType edge_function = [&](const cv::Point& a, const cv::Point& b)
    {
        return cv::norm(img(a), img(b)) > 50;
    };
    std::vector<Label> labels = createlabels(img.size(), edge_function);

    printf("labels.size(): %lu\n", labels.size());
    fflush(stdout);

    cv::waitKey();
    return 0;
}
