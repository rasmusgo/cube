#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

int idToCubie(int id);

std::vector<size_t> assignColorsToSides(const std::vector<cv::Scalar>& colors);
