#include "MatrixTools.hpp"

#include <opencv2/opencv.hpp>

void printMat1d(const cv::Mat1d& mat)
{
    for (int i = 0; i < mat.rows; ++i)
    {
        printf("   ");
        for (int j = 0; j < mat.cols; ++j)
        {
            printf(" %5.2f", mat(i, j));
        }
        printf("\n");
    }
}

void printMatx33d(const cv::Matx33d& mat)
{
    printf("    %5.2f %5.2f %5.2f\n    %5.2f %5.2f %5.2f\n    %5.2f %5.2f %5.2f\n",
        mat(0,0), mat(0,1), mat(0,2),
        mat(1,0), mat(1,1), mat(1,2),
        mat(2,0), mat(2,1), mat(2,2));
}

cv::Matx<double, 3, 9> rodriguesJacobian(const cv::Matx33d& rmat)
{
    cv::Matx<double, 9, 3> Jt;
    cv::Vec3d dummy_vec;
    cv::Rodrigues(rmat, dummy_vec, Jt);
    return Jt.t();
}

cv::Matx<double, 9, 3> rodriguesJacobian(const cv::Vec3d& rvec)
{
    cv::Matx<double, 3, 9> Jt;
    cv::Matx33d dummy_mat;
    cv::Rodrigues(rvec, dummy_mat, Jt);
    return Jt.t();
}

cv::Matx33d closest90DegreeRotation(const cv::Matx33d& in_rotation)
{
    cv::Matx<uint8_t, 3,3> mask = cv::Matx<uint8_t, 3,3>::all(255);
    cv::Matx33d out_rotation = cv::Matx33d::zeros();

    for (int i = 0; i < 3; ++i)
    {
        double min_val;
        double max_val;
        cv::Point min_loc;
        cv::Point max_loc;
        cv::minMaxLoc(in_rotation, &min_val, &max_val, &min_loc, &max_loc, mask);
        const cv::Point absmax_loc = std::abs(min_val) > std::abs(max_val) ? min_loc : max_loc;

        out_rotation(absmax_loc.y, absmax_loc.x) =
            in_rotation(absmax_loc.y, absmax_loc.x) > 0.0 ? 1.0 : -1.0;

        for (int j = 0; j < 3; ++j)
        {
            mask(absmax_loc.y, j) = 0;
            mask(j, absmax_loc.x) = 0;
        }
    }

    return out_rotation;
}
