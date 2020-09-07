#ifndef IMG_PROCESSING
#define IMG_PROCESSING

#include <opencv2/opencv.hpp>

void perspective_warp(cv::Mat &input, cv::Mat &warped);

void sobel_filtering(cv::Mat &warped, cv::Mat &sobel_line, int sobel_min = 10, int sobel_max = 170, int sobel_threshold = 0);
void sobel_abs_thresh(const cv::Mat &src, cv::Mat &dst, int dx, int dy, int thresh_min, int thresh_max, int kernel_size = 3);

void lane_histogram(cv::Mat &line, cv::Mat &histogram);


#endif
