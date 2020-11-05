#include "drive_calculation.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>

int calc_speed (std::vector<cv::Point> & midpoints) {

  int abs_diff = 0;
  std::cout << midpoints.size() << std::endl;

  // (midpoints.size()-1) in for loop would be 4 billion.. sth if midpoints.size() equals 0
  if (midpoints.size() < 2) return min_speed;

  for(int i = 0; i < (int)midpoints.size()-1; i++) {
    abs_diff += abs(midpoints[i].x - midpoints[i+1].x);
  }
  std::cout << abs_diff << std::endl;

  if (abs_diff == 0) return min_speed; // dont divide by 0
  
  int speed = min_speed + k_speed / abs_diff;
  return (speed > 255) ? 255 : speed;
}

float calc_angle (cv::Mat & img, std::vector<cv::Point> & midpoints, bool draw_steerpoint) {
  cv::Point steerpoint(0,0);
  for(int i = 0; i < n_steer_points; i++) {
    steerpoint += midpoints[i] * steer_points_weight[i];
  }
  if(draw_steerpoint) {
    cv::circle(img, steerpoint, 2, cv::Scalar(0,0,255), 2);
    cv::line(img, cv::Point(img.cols/2, img.rows), steerpoint, cv::Scalar(255,20,0));
  }
  //if(steerpoint.x < img.cols/2) {
    //return atan((img.rows - steerpoint.y)/(img.cols/2 - steerpoint.x));
  //} else {
  std::cout << "Gegegnk " << img.rows - steerpoint.y << " ank " << (img.cols/2 - steerpoint.x) << std::endl;
  return atan((float)(steerpoint.x - img.cols/2)/(img.rows - steerpoint.y))*180/3.142;
  //}
}
