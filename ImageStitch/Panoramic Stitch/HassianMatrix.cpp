#include "HassianMatrix.h"

MatrixXf HassianMatrix(int image_width, int image_length)
{
	int n = image_width * image_length;
	MatrixXf HMatrix = MatrixXf::Zero(n, n);

	for (int i = 0; i < n - 1; i++) {
		int x = i % image_width;
		int y = i / image_width;

		if (x == image_length - 1) {
			int j = i + image_length;
			HMatrix(i, j) = 1;
		}
		else if (y == image_width - 1) {
			int j = i + 1;
			HMatrix(i, j) = 1;
		}
		else {
			int j = i + 1;
			int k = i + image_length;
			HMatrix(i, j) = 1;
			HMatrix(i, k) = 1;
		}
	}
	return HMatrix;
}