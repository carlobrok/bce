#ifndef DRIVE_CALCULATION
#define DRIVE_CALCULATION

#include <opencv2/opencv.hpp>
#include <vector>

const int min_speed = 50;
const double k_speed = 200;

int calc_speed(std::vector<cv::Point> & midpoints);

const int n_steer_points = 5;
const double steer_points_weight[n_steer_points] = {0.2,0.3,0.3,0.1,0.1};

float calc_angle(cv::Mat & img, std::vector<cv::Point> & midpoints, bool draw_steerpoint = false);

#endif
