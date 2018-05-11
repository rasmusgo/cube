#include "OpticalFlow.hpp"

#include <array>
#include <vector>

#include <opencv2/opencv.hpp>

#include "Timer.hpp"

namespace {

const size_t NUM_INNER_ITERATIONS = 2;
const size_t MIN_LEVEL = 1;
const size_t MAX_LEVEL = 4;
const size_t NUM_LEVELS = MAX_LEVEL + 1;
const float INVERSE_SOBEL_SCALE_FACTOR = 1.0f / 8.0f;

const size_t NUM_FEATURE_DIMENSIONS = 9;
using FeatureVec = cv::Vec<float, NUM_FEATURE_DIMENSIONS>;
using FeatureMat = cv::Mat_<FeatureVec>;

} // namespace

std::array<FeatureMat, NUM_LEVELS> createPyramid(const cv::Mat3b& image)
{
    std::array<FeatureMat, NUM_LEVELS> pyramid;

    // The first three channels are the color image.
    const cv::Mat3f image_float = cv::Mat3f(image) * (1.0f / 255.0f);

    // Three levels of difference of Gaussians
    cv::Mat3f image_g1;
    cv::Mat3f image_g2;
    cv::Mat3f image_g3;
    cv::Mat3f image_g4;
    const cv::Size kernel_size_g1(3, 3);
    const cv::Size kernel_size_g2(13, 13);
    const cv::Size kernel_size_g3(35, 35);
    const cv::Size kernel_size_g4(101, 101);
    cv::GaussianBlur(image_float, image_g1, kernel_size_g1, 0.0);
    cv::GaussianBlur(image_float, image_g2, kernel_size_g2, 0.0);
    cv::GaussianBlur(image_float, image_g3, kernel_size_g3, 0.0);
    cv::GaussianBlur(image_float, image_g4, kernel_size_g4, 0.0);

    // Merge the channels to form the image of feature vectors.
    const std::vector<cv::Mat> image_array = {
        image_g1 - image_g2,
        image_g2 - image_g3,
        image_g3 - image_g4,
    };
    std::vector<int> from_to;
    for (int i = 0; i < NUM_FEATURE_DIMENSIONS; ++i)
    {
        from_to.push_back(i);
        from_to.push_back(i);
    }
    pyramid[0] = FeatureMat(image.size());
    cv::mixChannels(image_array, pyramid[0], from_to.data(), NUM_FEATURE_DIMENSIONS);

    // Create a pyramid of feature images.
    for (size_t i = 1; i < pyramid.size(); ++i)
    {
        cv::pyrDown(pyramid[i - 1], pyramid[i]);
    }
    return pyramid;
}

cv::Mat3f part1FromFeatureVec(const FeatureMat& image)
{
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    channels.resize(3);
    cv::Mat3f out;
    cv::merge(channels, out);
    return out;
}

cv::Mat3f part2FromFeatureVec(const FeatureMat& image)
{
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    channels = {
        channels.at(3),
        channels.at(4),
        channels.at(5),
    };
    cv::Mat3f out;
    cv::merge(channels, out);
    return out;
}

cv::Mat3f part3FromFeatureVec(const FeatureMat& image)
{
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    channels = {
        channels.at(6),
        channels.at(7),
        channels.at(8),
    };
    cv::Mat3f out;
    cv::merge(channels, out);
    return out;
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
    const float regularization_towards_zero = 0.001f;
    const float regularization_towards_same = 0.01f;
    const float regularization_towards_smooth = 0.1f;
    cv::Mat2f flow_dx;
    cv::Mat2f flow_dy;
    cv::Sobel(flow, flow_dx, CV_32F, 1, 0, 3, INVERSE_SOBEL_SCALE_FACTOR);
    cv::Sobel(flow, flow_dy, CV_32F, 0, 1, 3, INVERSE_SOBEL_SCALE_FACTOR);

    cv::Mat2f flow_update(delta.size());
    cv::Mat1f uncertainty(delta.size());
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
            uncertainty(row, col) = std::sqrt(JtJinv(0,0) + JtJinv(1,1));
        }
    }
    flow -= flow_update;

    const FeatureMat intermediate = (image_from_b + image_from_a) * 0.5f;
    {
        char label[] = "intermediate[0] part1";
        label[13] = '0' + i;
        cv::imshow(label, 0.5f + part1FromFeatureVec(intermediate) * 0.5f);
    }
    {
        char label[] = "intermediate[0] part2";
        label[13] = '0' + i;
        cv::imshow(label, 0.5f + part2FromFeatureVec(intermediate) * 0.5f);
    }
    {
        char label[] = "intermediate[0] part3";
        label[13] = '0' + i;
        cv::imshow(label, 0.5f + part3FromFeatureVec(intermediate) * 0.5f);
    }

    {
        char label[] = "delta[0] part1";
        label[6] = '0' + i;
        cv::imshow(label, 0.5f + part1FromFeatureVec(delta) * 0.5f);
    }

    {
        char label[] = "delta[0] part2";
        label[6] = '0' + i;
        cv::imshow(label, 0.5f + part2FromFeatureVec(delta) * 0.5f);
    }

    {
        char label[] = "delta[0] part3";
        label[6] = '0' + i;
        cv::imshow(label, 0.5f + part3FromFeatureVec(delta) * 0.5f);
    }

    {
        char label[] = "delta_dx[0]";
        label[9] = '0' + i;
        cv::imshow(label, 0.5f + part1FromFeatureVec(delta_dx) * 0.2f);
    }

    {
        char label[] = "delta_dy[0]";
        label[9] = '0' + i;
        cv::imshow(label, 0.5f + part1FromFeatureVec(delta_dy) * 0.2f);
    }

    {
        char label[] = "flow[0]";
        label[5] = '0' + i;
        cv::Mat1f xy[2];
        cv::split(flow, xy);
        cv::Mat3f xyy;
        cv::merge(std::vector<cv::Mat>{xy[0], xy[1], xy[1]}, xyy);
        const float scale = 20.0f / delta.cols;
        cv::imshow(label, 0.5 + xyy * scale);
    }

    {
        char label[] = "uncertainty[0]";
        label[12] = '0' + i;
        cv::imshow(label, uncertainty * 0.1f);
    }
}

cv::Mat2f intermediateOpticalFlow(
    const cv::Mat3b& a,
    const cv::Mat3b& b)
{
    assert(!a.empty());
    assert(!b.empty());
    assert(a.cols == b.cols);
    assert(a.rows == b.rows);

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

    return flow_pyramid[MIN_LEVEL];
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
