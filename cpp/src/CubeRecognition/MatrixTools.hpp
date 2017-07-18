#pragma once

#include <opencv2/core/mat.hpp>

void printMat1d(const cv::Mat1d& mat);
void printMatx33d(const cv::Matx33d& mat);

cv::Matx<double, 3, 9> rodriguesJacobian(const cv::Matx33d& rmat);
cv::Matx<double, 9, 3> rodriguesJacobian(const cv::Vec3d& rvec);

cv::Matx33d closest90DegreeRotation(const cv::Matx33d& in_rotation);
