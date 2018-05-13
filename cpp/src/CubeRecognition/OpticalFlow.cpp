#include "OpticalFlow.hpp"

#include <array>
#include <cstdlib>
#include <vector>

#include <opencv2/opencv.hpp>

#include "Timer.hpp"

namespace {

const size_t NUM_INNER_ITERATIONS = 3;
const size_t MIN_LEVEL = 0;
const size_t MAX_LEVEL = 5;
const size_t NUM_LEVELS = MAX_LEVEL + 1;
const float INVERSE_SOBEL_SCALE_FACTOR = 1.0f / 8.0f;

const size_t NUM_FEATURE_DIMENSIONS = 3;
using FeatureVec = cv::Vec<float, NUM_FEATURE_DIMENSIONS>;
using FeatureMat = cv::Mat_<FeatureVec>;

} // namespace

std::array<FeatureMat, NUM_LEVELS> createPyramid(const cv::Mat3b& image)
{
    std::array<FeatureMat, NUM_LEVELS> pyramid;

    // The first three channels are the color image.
    const cv::Mat3f image_float = cv::Mat3f(image) * (1.0f / 255.0f);

    // Create a pyramid of images.
    pyramid[0] = image_float;
    for (size_t i = 1; i < pyramid.size(); ++i)
    {
        cv::pyrDown(pyramid[i - 1], pyramid[i]);
    }

    // Replace images with difference of Gaussians.
    const cv::Size kernel_size_g1(3, 3);
    const cv::Size kernel_size_g2(9, 9);
    for (size_t i = 0; i < pyramid.size(); ++i)
    {
        cv::Mat3f image_g1;
        cv::Mat3f image_g2;
        cv::GaussianBlur(pyramid[i], image_g1, kernel_size_g1, 0.0);
        cv::GaussianBlur(pyramid[i], image_g2, kernel_size_g2, 0.0);
        pyramid[i] = image_g1 - image_g2;
    }

    return pyramid;
}

cv::Mat3f packPyramid(const std::array<cv::Mat3f, NUM_LEVELS>& pyramid)
{
    const cv::Size canvas_size(
        pyramid[0].cols + pyramid[1].cols,
        pyramid[0].rows);
    cv::Mat3f canvas(canvas_size, cv::Vec3f(0, 0, 0));

    pyramid[0].copyTo(canvas(cv::Rect(0, 0, pyramid[0].cols, pyramid[0].rows)));
    const int x = pyramid[0].cols;
    int y = 0;
    for (int i = 1; i < NUM_LEVELS; ++i)
    {
        const int w = pyramid[i].cols;
        const int h = pyramid[i].rows;
        pyramid[i].copyTo(canvas(cv::Rect(x, y, w, h)));
        y += h;
    }
    return canvas;
}

cv::Mat3f packPyramids(
    const std::array<cv::Mat3f, NUM_LEVELS>& top_left,
    const std::array<cv::Mat3f, NUM_LEVELS>& top_right,
    const std::array<cv::Mat3f, NUM_LEVELS>& bottom_left)
{
    assert(!top_left[0].empty());
    assert(!top_right[0].empty());
    assert(!bottom_left[0].empty());

    for (size_t i = 0; i < NUM_LEVELS; ++i)
    {
        assert(top_left[i].size() == top_right[i].size());
        assert(top_left[i].size() == bottom_left[i].size());
    }

    cv::Size max_size(0, 0);
    {
        int x = 0;
        int y = 0;
        for (size_t i = 0; i < NUM_LEVELS; ++i)
        {
            const int w = top_left[i].cols;
            const int h = top_left[i].rows;
            x += w;
            y += h;
            max_size.width = std::max(max_size.width, x + w);
            max_size.height = std::max(max_size.height, y + h);
        }
    }

    cv::Mat3f canvas(max_size, cv::Vec3f(0, 0, 0));

    int x = 0;
    int y = 0;
    for (size_t i = 0; i < NUM_LEVELS; ++i)
    {
        const int w = top_left[i].cols;
        const int h = top_left[i].rows;
        top_left[i].copyTo(canvas(cv::Rect(x, y, w, h)));
        top_right[i].copyTo(canvas(cv::Rect(x + w, y, w, h)));
        bottom_left[i].copyTo(canvas(cv::Rect(x, y + h, w, h)));
        x += w;
        y += h;
    }
    return canvas;
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

void innerLoop(const FeatureMat& a, const FeatureMat& b, cv::Mat2f& flow, int i)
{
    // Compute lookup tables
    const cv::Mat2f map_a_from_mid = createAbsoluteMapFromRelative(-flow);
    const cv::Mat2f map_b_from_mid = createAbsoluteMapFromRelative(flow);

    // Compute gradients for a
    FeatureMat a_dx;
    FeatureMat a_dy;
    cv::Sobel(a, a_dx, CV_32F, 1, 0, 3, INVERSE_SOBEL_SCALE_FACTOR);
    cv::Sobel(a, a_dy, CV_32F, 0, 1, 3, INVERSE_SOBEL_SCALE_FACTOR);

    // Map image and gradients from image a into intermediate image
    FeatureMat dx_from_a;
    cv::remap(a_dx, dx_from_a, map_a_from_mid, cv::noArray(), cv::INTER_LINEAR);
    FeatureMat dy_from_a;
    cv::remap(a_dy, dy_from_a, map_a_from_mid, cv::noArray(), cv::INTER_LINEAR);
    FeatureMat image_from_a;
    cv::remap(a, image_from_a, map_a_from_mid, cv::noArray(), cv::INTER_LINEAR);

    // Compute gradients for b
    FeatureMat b_dx;
    FeatureMat b_dy;
    cv::Sobel(b, b_dx, CV_32F, 1, 0, 3, INVERSE_SOBEL_SCALE_FACTOR);
    cv::Sobel(b, b_dy, CV_32F, 0, 1, 3, INVERSE_SOBEL_SCALE_FACTOR);

    // Map image and gradients from image b into intermediate image
    FeatureMat dx_from_b;
    cv::remap(b_dx, dx_from_b, map_b_from_mid, cv::noArray(), cv::INTER_LINEAR);
    FeatureMat dy_from_b;
    cv::remap(b_dy, dy_from_b, map_b_from_mid, cv::noArray(), cv::INTER_LINEAR);
    FeatureMat image_from_b;
    cv::remap(b, image_from_b, map_b_from_mid, cv::noArray(), cv::INTER_LINEAR);

    // Estimate flow
    const FeatureMat delta = FeatureMat(image_from_b) - FeatureMat(image_from_a);
    const FeatureMat delta_dx = FeatureMat(dx_from_b) + FeatureMat(dx_from_a);
    const FeatureMat delta_dy = FeatureMat(dy_from_b) + FeatureMat(dy_from_a);

    // Compute gradients for flow regularization
    const float regularization_towards_zero   = 0.00f;
    const float regularization_towards_same   = 0.01f;
    const float regularization_towards_smooth = 0.01f;
    cv::Mat2f flow_dx;
    cv::Mat2f flow_dy;
    cv::Sobel(flow, flow_dx, CV_32F, 1, 0, 3, INVERSE_SOBEL_SCALE_FACTOR);
    cv::Sobel(flow, flow_dy, CV_32F, 0, 1, 3, INVERSE_SOBEL_SCALE_FACTOR);

    cv::Mat2f flow_update(delta.size());
    //cv::Mat1f uncertainty(delta.size());
    #pragma omp for
    for (int row = 0; row < delta.rows; ++row)
    {
        for (int col = 0; col < delta.cols; ++col)
        {
            const auto& d = delta(row, col);
            const auto& dx = delta_dx(row, col);
            const auto& dy = delta_dy(row, col);

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
            const float dxdx = dx.dot(dx);
            const float dxdy = dx.dot(dy);
            const float dydy = dy.dot(dy);

            const float regularization =
                regularization_towards_zero +
                regularization_towards_same +
                2 * regularization_towards_smooth;
            const cv::Matx22f JtJ(
                dxdx + regularization, dxdy,
                dxdy, dydy + regularization);
            const cv::Matx22f JtJinv = JtJ.inv();

            const cv::Vec2f Jtd = cv::Vec2f(dx.dot(d), dy.dot(d))
                + regularization_towards_zero * flow(row, col)
                + regularization_towards_smooth * flow_dx(row, col)
                + regularization_towards_smooth * flow_dy(row, col);

            flow_update(row, col) = JtJinv * Jtd;
            //uncertainty(row, col) = std::sqrt(JtJinv(0,0) + JtJinv(1,1));
        }
    }
    flow -= flow_update;

    /*
    {
        char label[] = "delta[0]";
        label[6] = '0' + i;
        cv::imshow(label, 0.5f + delta * 0.5f);
    }

    {
        char label[] = "delta_dx[0]";
        label[9] = '0' + i;
        cv::imshow(label, 0.5f + delta_dx * 0.2f);
    }

    {
        char label[] = "delta_dy[0]";
        label[9] = '0' + i;
        cv::imshow(label, 0.5f + delta_dy * 0.2f);
    }

    {
        char label[] = "flow[0]";
        label[5] = '0' + i;
        cv::imshow(label, colorizeOpticalFlow(flow));
    }

    {
        char label[] = "uncertainty[0]";
        label[12] = '0' + i;
        cv::imshow(label, uncertainty * 0.1f);
    }
    */
}

cv::Mat2f intermediateOpticalFlow(
    const cv::Mat3b& a,
    const cv::Mat3b& b)
{
    assert(!a.empty());
    assert(!b.empty());
    assert(a.cols == b.cols);
    assert(a.rows == b.rows);

    cv::imshow("a", a);
    cv::imshow("b", b);

    startTimer();
    const auto a_pyramid = createPyramid(a);
    const auto b_pyramid = createPyramid(b);
    stopTimer("creating pyramids");

    startTimer();
    // Compute flow starting from the smallest images in the pyramid.
    std::array<cv::Mat2f, NUM_LEVELS> flow_pyramid;
    for (int i = MAX_LEVEL; i >= int(MIN_LEVEL); --i)
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
        for (int j = 0; j < NUM_INNER_ITERATIONS; ++j)
        {
            innerLoop(a_pyramid[i], b_pyramid[i], flow_pyramid[i], i);
        }
    }
    stopTimer("computing flow");

    startTimer();
    std::array<cv::Mat3f, NUM_LEVELS> colorized_flow_pyramid;
    std::array<cv::Mat3f, NUM_LEVELS> mid_pyramid;
    for (size_t i = 0; i < NUM_LEVELS; ++i)
    {
        const auto& flow = flow_pyramid[i];
        colorized_flow_pyramid[i] = colorizeOpticalFlow(flow);
        const cv::Mat2f map_a_from_mid = createAbsoluteMapFromRelative(-flow);
        const cv::Mat2f map_b_from_mid = createAbsoluteMapFromRelative(flow);
        cv::Mat3f image_from_a;
        cv::Mat3f image_from_b;
        cv::remap(a_pyramid[i], image_from_a, map_a_from_mid, cv::noArray(), cv::INTER_LINEAR);
        cv::remap(b_pyramid[i], image_from_b, map_b_from_mid, cv::noArray(), cv::INTER_LINEAR);
        mid_pyramid[i] = (image_from_a + image_from_b) * 0.5f;
    }
    cv::imshow("feature pyramids", 0.5f + packPyramids(a_pyramid, b_pyramid, mid_pyramid) * 0.5f);
    cv::imshow("flow pyramid", packPyramid(colorized_flow_pyramid));
    stopTimer("visualizing flow");

    return flow_pyramid[MIN_LEVEL];
}

cv::Mat3f colorizeOpticalFlow(const cv::Mat2f& flow)
{
    cv::Mat1f xy[2];
    cv::split(flow, xy);
    cv::Mat3f xyy;
    cv::merge(std::vector<cv::Mat>{xy[0], xy[1] * 0, xy[1]}, xyy);
    const float scale = 20.0f / std::max(flow.cols, flow.rows);
    return 0.5 + xyy * scale;
}
