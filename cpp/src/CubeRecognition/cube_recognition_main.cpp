#include <cstdio>
#include <cstdlib>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ConnectLabels.hpp"
#include "CreateLabels.hpp"
#include "Image.hpp"
#include "Label.hpp"

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

int main()
{
    cv::Mat3b img = cv::imread("photos/IMG_6217.JPG", cv::IMREAD_COLOR);
    struct WinPos
    {
        std::string name;
        int x;
        int y;
    };
    std::vector<WinPos> window_positions = {
        {"labels", 0, 0},
        {"debug labels", img.cols + 10, 0},
        {"F", 0, img.rows},
        {"R", img.cols / 3, img.rows},
        {"U", 2 * img.cols / 3, img.rows},
    };
    for (auto w : window_positions)
    {
        cv::namedWindow(w.name);
        cv::moveWindow(w.name, w.x, w.y);
    }

    for (auto threshold : {4, 6, 8, 10, 12, 14, 16})
    {
        EdgeFunctionType edge_function = [&](const cv::Point& a, const cv::Point& b)
        {
            return cv::norm(img(a), img(b)) > threshold;
        };
        std::vector<Label> labels = createlabels(img.size(), edge_function);

        printf("Threshold: %d, Num labels: %lu\n", threshold, labels.size());
        fflush(stdout);

        cv::Mat3b canvas(img.size(), cv::Vec3b(0,0,0));
        for (auto label : labels)
        {
            drawLabel(canvas, label, cv::Scalar(255, 255, 255));
        }
        cv::imshow("labels", canvas);

        // Connect labels with each other in order to associate them.
        connectLabels(labels);

        cv::waitKey();
    }

    return 0;
}
