#include "OpticalFlow.hpp"

#include <array>
#include <vector>

#include <opencv2/opencv.hpp>

namespace {

const size_t NUM_LEVELS = 5;
const size_t MAX_LEVEL = NUM_LEVELS - 1;
const float INVERSE_SOBEL_SCALE_FACTOR = 1.0f / 8.0f;

} // namespace

std::array<cv::Mat3b, NUM_LEVELS> createPyramid(const cv::Mat3b& image)
{
    std::array<cv::Mat3b, NUM_LEVELS> pyramid;
    pyramid[0] = image;
    for (size_t i = 1; i < pyramid.size(); ++i)
    {
        cv::pyrDown(pyramid[i - 1], pyramid[i]);
    }
    return pyramid;
}

cv::Mat2f createAbsoluteMapFromRelative(const cv::Mat2f& offset)
{
    cv::Mat2f mapping(offset.size());
    for (int row = 0; row < offset.rows; ++row)
    {
        for (int col = 0; col < offset.cols; ++col)
        {
            const auto& d = offset(row, col);
            mapping(row, col) = cv::Vec2f(col + d[0], row + d[1]);
        }
    }
    return mapping;
}

cv::Mat2f intermediateOpticalFlow(
    const cv::Mat3b& a,
    const cv::Mat3b& b)
{
    assert(!a.empty());
    assert(!b.empty());
    assert(a.cols == b.cols);
    assert(a.rows == b.rows);

    const auto a_pyramid = createPyramid(a);
    const auto b_pyramid = createPyramid(b);

    std::array<cv::Mat2f, NUM_LEVELS> flow_pyramid;

    // Compute flow starting from the smallest images in the pyramid.
    for (int i = MAX_LEVEL; i >= 0; --i)
    {
        const cv::Size target_size = a_pyramid[i].size();
        if (i == MAX_LEVEL)
        {
            flow_pyramid[i] = cv::Mat2d::zeros(target_size);
        }
        else
        {
            cv::pyrUp(flow_pyramid[i + 1], flow_pyramid[i], target_size);
            flow_pyramid[i] *= 2.0f;
        }

        // Compute lookup tables
        const cv::Mat2f map_a_from_mid = createAbsoluteMapFromRelative(-flow_pyramid[i]);
        const cv::Mat2f map_b_from_mid = createAbsoluteMapFromRelative(flow_pyramid[i]);

        // Compute gradients for a
        cv::Mat3f a_dx;
        cv::Mat3f a_dy;
        cv::Sobel(a_pyramid[i], a_dx, CV_32F, 1, 0, 3, INVERSE_SOBEL_SCALE_FACTOR);
        cv::Sobel(a_pyramid[i], a_dy, CV_32F, 0, 1, 3, INVERSE_SOBEL_SCALE_FACTOR);

        // Map image and gradients from image a into intermediate image
        cv::Mat3f dx_from_a;
        cv::remap(a_dx, dx_from_a, map_a_from_mid, cv::noArray(), cv::INTER_LINEAR);
        cv::Mat3f dy_from_a;
        cv::remap(a_dy, dy_from_a, map_a_from_mid, cv::noArray(), cv::INTER_LINEAR);
        cv::Mat3b image_from_a;
        cv::remap(a_pyramid[i], image_from_a, map_a_from_mid, cv::noArray(), cv::INTER_LINEAR);

        // Compute gradients for b
        cv::Mat3f b_dx;
        cv::Mat3f b_dy;
        cv::Sobel(b_pyramid[i], b_dx, CV_32F, 1, 0, 3, INVERSE_SOBEL_SCALE_FACTOR);
        cv::Sobel(b_pyramid[i], b_dy, CV_32F, 0, 1, 3, INVERSE_SOBEL_SCALE_FACTOR);

        // Map image and gradients from image b into intermediate image
        cv::Mat3f dx_from_b;
        cv::remap(b_dx, dx_from_b, map_b_from_mid, cv::noArray(), cv::INTER_LINEAR);
        cv::Mat3f dy_from_b;
        cv::remap(b_dy, dy_from_b, map_b_from_mid, cv::noArray(), cv::INTER_LINEAR);
        cv::Mat3b image_from_b;
        cv::remap(b_pyramid[i], image_from_b, map_b_from_mid, cv::noArray(), cv::INTER_LINEAR);

        // Estimate flow
        const cv::Mat3f delta = cv::Mat3f(image_from_b) - cv::Mat3f(image_from_a);
        const cv::Mat3f delta_dx = cv::Mat3f(dx_from_b) + cv::Mat3f(dx_from_a);
        const cv::Mat3f delta_dy = cv::Mat3f(dy_from_b) + cv::Mat3f(dy_from_a);

        for (int row = 0; row < delta.rows; ++row)
        {
            for (int col = 0; col < delta.cols; ++col)
            {
                const auto& d = delta(row, col);
                const auto& dx = delta_dx(row, col);
                const auto& dy = delta_dy(row, col);

                const cv::Matx32f J(
                    dx[0], dy[0],
                    dx[1], dy[1],
                    dx[2], dy[2]);
                /*
                h_2_1
                d_3_1
                dx_3_1
                dy_3_1
                J_3_2 = (dx_3_1 | dy_3_1)

                0 = d_3_1 + J_3_2 * h_2_1
                0 = Jt_2_3 * d_3_1 + Jt_2_3 * J_3_2 * h_2_1
                (Jt_2_3 * J_3_2).inv() * -Jt_2_3 * d_3_1 = h_2_1
                - JtJ_2_2.inv() * Jt_2_3 * d_3_1 = h_2_1
                */
                const float reg = 1.0e-3f;
                const cv::Matx22f regularization(
                    reg, 0.0f,
                    0.0f, reg);
                const cv::Matx22f JtJ = J.t() * J + regularization;
                flow_pyramid[i](row, col) -= (JtJ).inv() * J.t() * d;
            }
        }

        {
            char label[] = "delta[0]";
            label[6] = '0' + i;
            cv::imshow(label, 0.5 + delta * 0.01);
        }

        {
            char label[] = "delta_dx[0]";
            label[9] = '0' + i;
            cv::imshow(label, 0.5 + delta_dx * 0.01);
        }

        {
            char label[] = "delta_dy[0]";
            label[9] = '0' + i;
            cv::imshow(label, 0.5 + delta_dy * 0.01);
        }

        {
            char label[] = "flow_x[0]";
            label[7] = '0' + i;
            cv::Mat1f xy[2];
            cv::split(flow_pyramid[i], xy);
            const float scale = 10.0f / delta.cols;
            cv::imshow(label, 0.5 + xy[0] * scale);
            label[5] = 'y';
            cv::imshow(label, 0.5 + xy[1] * scale);
        }
    }
    return flow_pyramid[0];
}

cv::Mat3f colorizeOpticalFlow(const cv::Mat2f& flow)
{
    cv::Mat1f xy[2]; // X,Y
    cv::split(flow, xy);

    // Calculate angle and magnitude
    cv::Mat1f magnitude;
    cv::Mat1f angle;
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

    magnitude *= 50.0 / flow.cols;

    // Build hsv image
    cv::Mat1f _hsv[3];
    cv::Mat3f hsv;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magnitude;
    cv::merge(_hsv, 3, hsv);

    // Convert to BGR and show
    cv::Mat3f bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    return bgr;
}
