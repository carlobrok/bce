#include "line_calculation.hpp"

#include <opencv2/opencv.hpp>

void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order) {
	CV_Assert((src_x.rows > 0) && (src_y.rows > 0) && (src_x.cols == 1) && (src_y.cols == 1)
		&& (dst.cols == 1) && (dst.rows == (order + 1)) && (order >= 1));
	cv::Mat X;
	X = cv::Mat::zeros(src_x.rows, order + 1, CV_32FC1);
	cv::Mat copy;
	for (int i = 0; i <= order; i++)
	{
		copy = src_x.clone();
		pow(copy, i, copy);
		cv::Mat M1 = X.col(i);
		copy.col(0).copyTo(M1);
	}
	cv::Mat X_t, X_inv;
	transpose(X, X_t);
	cv::Mat temp = X_t * X;
	cv::Mat temp2;
	invert(temp, temp2);
	cv::Mat temp3 = temp2 * X_t;
	cv::Mat W = temp3 * src_y;
	W.copyTo(dst);
}
