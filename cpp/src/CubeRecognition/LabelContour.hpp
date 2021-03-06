// The labels use four coordinate systems.
// They are classified based on what coordinate system gives the tightest bounding box.
//
//                                     .*.
//    type_f: (x, ym)               .*     *.
//    type_r: (x, yp)             *.    U    .*      .-----.
//    type_u: (yp, ym)            |  *.   .*  |      |  s  |
//    type_s: (x, y)              |  F  *  R  |      *-----*
//                                *.    |    .*
//                                   *. | .*
//                                      *
// typ_s (square) is not used, those candidates are discarded.

//             min_y
//     min_yp _  A  _ min_ym
//           |\  |  /|
//             \ | /
//              \|/
//  min_x <------+-------> max_x
//              /|\
//             / | \
//           |/_ | _\|
//     max_ym    V    max_yp
//             max_y

#pragma once

#include <opencv2/core/mat.hpp>

#include <vector>

#include "Settings.hpp"

template <typename T>
T xy2yp(T x, T y) {
    return y + x * KP;
}

template <typename T>
T xy2ym(T x,T y) {
    return y - x * KM;
}

template <typename T>
T xyp2y(T x,T yp) {
    return yp - x * KP;
}

template <typename T>
T xym2y(T x,T ym) {
    return ym + x * KM;
}

template <typename T>
T ypym2x(T yp,T ym) {
    return (yp- ym)/(KP+KM);
}

template <typename T>
T ypym2y(T yp,T ym) {
    return (yp*KM + ym*KP)/(KP+KM);
}

template <typename T>
T ypym2area(T yp,T ym) {
    return (yp * ym) / (KP+KM);
}

template <class B, class A>
std::vector<B> cast(const std::vector<A>& a_vec)
{
    std::vector<B> b_vec;
    b_vec.reserve(a_vec.size());
    for (const A& a : a_vec)
    {
        b_vec.push_back(static_cast<B>(a));
    }
    return b_vec;
}

struct LabelContour {
    using Points   = std::vector<cv::Point>;
    using Points2f = std::vector<cv::Point2f>;
    cv::Point2f center;
    cv::Vec<float, 8> size;
    float area;
    int type;
    float qmax;
    cv::Point2f native;
    cv::Rect2f native_rect;
    std::vector<cv::Point> contour_points;

    LabelContour(const cv::Point2f& center, float area, const cv::Vec<float, 8>& size,
        std::vector<cv::Point> contour_points);
    std::vector<cv::Point> guessneighbors();
    cv::Point2f native2xy(float nx, float ny) const;
    cv::Point2f xy2native(float x, float y) const;
    cv::Point2f native2xy(cv::Point2f p) const { return native2xy(p.x, p.y); }
    cv::Point2f xy2native(cv::Point2f p) const { return xy2native(p.x, p.y); }
    Points native2xy(const Points& vec) const;
    Points xy2native(const Points& vec) const;
    Points2f native2xy(const Points2f& vec) const;
    Points2f xy2native(const Points2f& vec) const;
};

bool are_neighbors(const LabelContour& a, const LabelContour& b);
