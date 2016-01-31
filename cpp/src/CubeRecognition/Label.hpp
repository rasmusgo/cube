// The labels use four coordinate systems.
// They are classified based on what coordinate system gives the tightest bounding box.
//
//                                     .*.
//    typ_x: (x, ym)                .*  U  *.
//    typ_y: (x, yp)              *.   (z)   .*      .-----.
//    typ_z: (yp, ym)             |  *.   .*  |      | (s) |
//    typ_s: (x, y)               |  F  *  R  |      *-----*
//                                *.(x) | (y).*
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

#include <vector>

#include "Settings.hpp"
#include "VectorMath.hpp"

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

struct Label {
    Real x;
    Real y;
    Real area;
    Real size[8];
    int type;
    Real qmax;
    bool used_in_grid;
    Vec2r native;

    Label(Real x, Real y, Real area, Real size[8]);
    std::vector<Vec2r> guessneighbors();
    Vec2r native2xy(Real nx, Real ny);
    Vec2r xy2native(Real x, Real y);
};
