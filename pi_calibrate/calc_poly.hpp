#ifndef CALC_POLY
#define CALC_POLY

#include "opencv2/opencv.hpp"
#include <vector>

inline std::vector<cv::Point> warp_poly_points;

void calc_transform(cv::Mat & input);

#endif
