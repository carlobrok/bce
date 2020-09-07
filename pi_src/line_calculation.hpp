#ifndef LINE_CALCULATION
#define LINE_CALCULATION

#include <opencv2/opencv.hpp>

void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order);

#endif
