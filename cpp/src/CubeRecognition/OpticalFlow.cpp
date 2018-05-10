#include "OpticalFlow.hpp"

#include <array>
#include <vector>

#include <opencv2/opencv.hpp>

namespace {

const size_t NUM_LEVELS = 5;
const size_t MAX_LEVEL = NUM_LEVELS - 1;
const float INVERSE_SOBEL_SCALE_FACTOR = 1.0f / 8.0f;

const size_t NUM_FEATURE_DIMENSIONS = 8;
using FeatureVec = cv::Vec<float, NUM_FEATURE_DIMENSIONS>;
using FeatureMat = cv::Mat_<FeatureVec>;
const float DERIVATIVES_WEIGHT = 10.0f;

} // namespace

std::array<FeatureMat, NUM_LEVELS> createPyramid(const cv::Mat3b& image)
{
    std::array<FeatureMat, NUM_LEVELS> pyramid;

    // The first three channels are the color image.
    const cv::Mat3f image_float = cv::Mat3f(image) * (1.0f / 255.0f);

    // Then the derivatives of intensity.
    cv::Mat1f image_gray;
    cv::cvtColor(image_float, image_gray, CV_BGR2GRAY);
    cv::Mat1f dx;
    cv::Mat1f dy;
    cv::Sobel(image_gray, dx, CV_32F, 1, 0, 3, INVERSE_SOBEL_SCALE_FACTOR * DERIVATIVES_WEIGHT);
    cv::Sobel(image_gray, dy, CV_32F, 0, 1, 3, INVERSE_SOBEL_SCALE_FACTOR * DERIVATIVES_WEIGHT);

    // Then the structure tensor.
    const cv::Size kernel_size(3,3);
    cv::Mat1f sxx = dx.mul(dx);
    cv::Mat1f sxy = dx.mul(dy);
    cv::Mat1f syy = dy.mul(dy);
    cv::blur(sxx, sxx, kernel_size);
    cv::blur(sxy, sxy, kernel_size);
    cv::blur(syy, syy, kernel_size);

    // Merge the channels to form the image of feature vectors.
    const std::vector<cv::Mat> image_array = {
        image_float,
        dx, dy,
        sxx, sxy, syy,
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

cv::Mat3f mat3fFromFeatureVec(const FeatureMat& image)
{
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    channels.resize(3);
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

    for (int row = 0; row < delta.rows; ++row)
    {
        for (int col = 0; col < delta.cols; ++col)
        {
            const auto& d = delta(row, col);
            const auto& dx = delta_dx(row, col);
            const auto& dy = delta_dy(row, col);

            cv::Matx<float, NUM_FEATURE_DIMENSIONS, 2> J;
            for (int i = 0; i < NUM_FEATURE_DIMENSIONS; ++i)
            {
                J(i, 0) = dx[i];
                J(i, 1) = dy[i];
            }
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
            const float reg = 0.01f;
            const cv::Matx22f regularization(
                reg, 0.0f,
                0.0f, reg);
            const cv::Matx22f JtJ = J.t() * J + regularization;
            flow(row, col) -= (JtJ).inv() * J.t() * d;
        }
    }

    {
        char label[] = "intermediate[0]";
        label[14] = '0' + i;
        const cv::Mat3f intermediate = (
            mat3fFromFeatureVec(image_from_b) +
            mat3fFromFeatureVec(image_from_a)) * 0.5f;
        cv::imshow(label, intermediate);
    }

    {
        char label[] = "delta[0]";
        label[6] = '0' + i;
        cv::imshow(label, 0.5f + mat3fFromFeatureVec(delta) * 0.5f);
    }

    {
        char label[] = "delta_dx[0]";
        label[9] = '0' + i;
        cv::imshow(label, 0.5f + mat3fFromFeatureVec(delta_dx) * 0.2f);
    }

    {
        char label[] = "delta_dy[0]";
        label[9] = '0' + i;
        cv::imshow(label, 0.5f + mat3fFromFeatureVec(delta_dy) * 0.2f);
    }

    {
        char label[] = "flow_x[0]";
        label[7] = '0' + i;
        cv::Mat1f xy[2];
        cv::split(flow, xy);
        const float scale = 10.0f / delta.cols;
        cv::imshow(label, 0.5 + xy[0] * scale);
        label[5] = 'y';
        cv::imshow(label, 0.5 + xy[1] * scale);
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
        for (int j = 0; j < 2; ++j)
        {
            innerLoop(a_pyramid[i], b_pyramid[i], flow_pyramid[i], i);
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
