#include <cstdio>
#include <cstdlib>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Image.hpp"

#include "Label.hpp"
#include "CreateLabels.hpp"

void drawLabel(cv::Mat& canvas, const Label& label, const cv::Scalar& color)
{
    Vec2r np = label.xy2native(label.x, label.y);

    // Find native size
    Real nsize[4][4] = {
        // Type 0: 0167 x,ym
        {label.size[0], label.size[1], label.size[6], label.size[7]},
        // Type 1: 0145 x,yp
        {label.size[0], label.size[1], label.size[4], label.size[5]},
        // Type 2: 4567 yp,ym
        {label.size[4], label.size[5], label.size[6], label.size[7]},
        // Type 3 (Square): 0123 x,y
        {label.size[0], label.size[1],label.size[2], label.size[3]},
    };

    Real snx = nsize[label.type][1] - nsize[label.type][0];
    Real sny = nsize[label.type][3] - nsize[label.type][2];
    Real snx2 = snx*0.5;
    Real sny2 = sny*0.5;

    std::vector<Vec2r> corners = {
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
    cv::Mat3b img = cv::imread("cube_0b.png", cv::IMREAD_COLOR);

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
        cv::waitKey();
    }

    return 0;
}
