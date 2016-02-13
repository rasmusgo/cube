#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

void bend(float &x, float &y, int size)
{
    // Transform the object by bending it as if it was bent in a elliptical way
    //      __
    //     / /
    //    / /
    //   | |
    //   | |
    //   | |
    //    \ \
    //     \_\
    //
    // The x-coordinate acts as the angle
    // The y-coordinate acts as the radius

    float a = ( y / size - 0.5 ) * M_PI_2;
    float d = ( x / size + 0.5 );
    x = -cos(a)*d*size + size*0.925;
    y = sin(a)*d*size + size*0.5;
}

void drawPolygon(cv::Mat& img, const cv::Mat1f& polygon, const cv::Scalar& color)
{
    std::vector<cv::Point> poly_points(polygon.rows);
    cv::Mat2f shallow_copy = polygon.reshape(2);
    std::transform(shallow_copy.begin(), shallow_copy.end(), poly_points.begin(), [](cv::Vec2f& p)
    {
        return cv::Point(p[0] * 256, p[1] * 256);
    });
    cv::fillPoly(img, std::vector<std::vector<cv::Point>>{poly_points}, color, cv::LINE_AA, 8);
}

// Generate move icons and store them on disk
int main()
{
    // Arrows are triangles and rectangles
    //       .           _           .
    //      / \     ^   | |         / \
    //     /   \    |   | |        /   \
    //    /     \   | arrowsize   / / \ \
    //   /       \  |   | |      / /| |\ \
    //  /_________\ v   | |     /_/ | | \_\
    //      | |         | |         | |
    //      | |         | |         | |
    //<---->| |         | |         | |
    //margin| |         | |         | |
    //      | |<------->| |         | |
    //      | | spacing | |         | |
    //      | |         | |         | |
    //      | |         | |         | |
    //      | |         | |         | |
    //    ->|_|<-       |_|         |_|
    //   linewidth

    int arrowsize = 100;
    int arrowspacing = 150;
    int margin_x = 120;
    int margin_y = 50;
    int length = 500;
    int linewidth = 30;
    int height = margin_y*2 + length;
    int width = margin_x*2 + linewidth;

    // The arrows are black on a white background
    cv::Scalar color{0, 0, 0};
    cv::Scalar color2{64, 64, 64};

    const int arrow_point_count = 5;
    cv::Mat1f arrow1(arrow_point_count,2);
    {
        int i = 0;
        arrow1(i,0) = margin_x + linewidth/2;            arrow1(i++,1) = margin_y - linewidth/2;
        arrow1(i,0) = margin_x;                          arrow1(i++,1) = margin_y;
        //arrow1(i,0) = margin_x - arrowsize/2;            arrow1(i++,1) = margin_y + arrowsize/2;
        arrow1(i,0) = margin_x - arrowsize;              arrow1(i++,1) = margin_y + arrowsize;
        //arrow1(i,0) = margin - arrowsize + linewidth;  arrow1(i++,1) = margin + arrowsize + linewidth;
        //arrow1(i,0) = margin;                          arrow1(i++,1) = margin + linewidth*2;
        //arrow1(i,0) = margin + linewidth;              arrow1(i++,1) = margin + linewidth*2;
        //arrow1(i,0) = margin + arrowsize;              arrow1(i++,1) = margin + arrowsize + linewidth;
        arrow1(i,0) = margin_x + linewidth + arrowsize;  arrow1(i++,1) = margin_y + arrowsize;
        //arrow1(i,0) = margin_x + linewidth + arrowsize/2;arrow1(i++,1) = margin_y + arrowsize/2;
        arrow1(i,0) = margin_x + linewidth;              arrow1(i++,1) = margin_y;
    }

    cv::Mat1f arrow2(arrow1.clone());
    for (int i = 0; i < arrow_point_count; ++i)
        arrow2(i,1) += arrowspacing;

    cv::Mat1f line(22,2);
    {
        int i = 0;
        int from = margin_y;
        int to = height - margin_y - 1;
        for (int j = 0; j <= 10; ++j) {
            line(i,0) = margin_x;               line(i++,1) = from + (to-from)*(j*0.1);
        }
        for (int j = 10; j >= 0; --j) {
            line(i,0) = margin_x + linewidth;   line(i++,1) = from + (to-from)*(j*0.1);
        }
    }

    // The arrows are first drawn straight with zero, one and two arrowheads
    // The arrow coordinates are then bent and different arrowheads are drawn
    // The images are then mirrored and rotated to create arrows in all desired directions

    cv::Mat1b bg(cv::Size(width,height), 255);

    cv::Mat1b img_R0(bg.clone());
    drawPolygon(img_R0, line, color2);

    cv::Mat1b img_R1(bg.clone());
    drawPolygon(img_R1, line, color);
    drawPolygon(img_R1, arrow1, color);

    cv::Mat1b img_R2(img_R1.clone());
    drawPolygon(img_R2, arrow2, color);

    cv::Mat1b img_R1m;
    cv::Mat1b img_R2m;
    cv::flip(img_R1, img_R1m, 0);
    cv::flip(img_R2, img_R2m, 0);

    cv::Mat1b img_U0(img_R0.t());
    cv::Mat1b img_U1(img_R1.t());
    cv::Mat1b img_U2(img_R2.t());
    cv::Mat1b img_U1m(img_R1m.t());
    cv::Mat1b img_U2m(img_R2m.t());

    // Bend the polygon arrows
    for (int i=0; i < arrow_point_count; ++i)
        bend(arrow1(i,0), arrow1(i,1), height);
    for (int i=0; i < arrow_point_count; ++i)
        bend(arrow2(i,0), arrow2(i,1), height);
    for (int i=0; i < 22; ++i)
        bend(line(i,0), line(i,1), height);

    cv::Mat1b img_F0(bg.clone());
    drawPolygon(img_F0, line, color2);

    cv::Mat1b img_F1m(bg.clone());
    drawPolygon(img_F1m, line, color);
    drawPolygon(img_F1m, arrow1, color);

    cv::Mat1b img_F2m(img_F1m.clone());
    drawPolygon(img_F2m, arrow2, color);

    img_F0 = img_F0.t();
    img_F1m = img_F1m.t();
    img_F2m = img_F2m.t();

    cv::Mat1b img_F1;
    cv::flip(img_F1m, img_F1, 1);
    cv::Mat1b img_F2;
    cv::flip(img_F2m, img_F2, 1);

    std::vector<cv::Mat1b> list{
        img_U2m,
        img_U1m,
        img_U0,
        img_U1,
        img_U2,
        img_R2m,
        img_R1m,
        img_R0,
        img_R1,
        img_R2,
        img_F2m,
        img_F1m,
        img_F0,
        img_F1,
        img_F2};

    const char name[][8] = {
        "U-2",
        "U-1",
        "U0",
        "U1",
        "U2",
        "R-2",
        "R-1",
        "R0",
        "R1",
        "R2",
        "F-2",
        "F-1",
        "F0",
        "F1",
        "F2",
    };

    for (int i = 0; i < 15; ++i) {
        // Generate filename
        char str[120];
        sprintf(str, "icons/move_%s.png", name[i]);
        //printf("Saving %s\n", str);

        // Save
        cv::Mat small;
        cv::resize(list[i], small, cv::Size(), 0.1, 0.1, cv::INTER_AREA);
        cv::imwrite(str, small);

        cv::imshow(name[i], small);
    }

    cv::waitKey(0);
    return 0;
}
