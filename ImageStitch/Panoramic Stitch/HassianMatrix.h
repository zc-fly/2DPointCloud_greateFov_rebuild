#pragma once

#include <Eigen/Core>

using namespace Eigen;

MatrixXf HassianMatrix(int image_width, int image_length);
