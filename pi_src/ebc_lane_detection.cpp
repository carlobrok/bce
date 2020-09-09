#include "devices.hpp"
#include "img_processing.hpp"
#include "line_calculation.hpp"
#include "VideoServer.h"
#include "CameraCapture.h"
#include "poly_regr.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>

const int num_images = 11;
//std::string images[num_images] = {};

int main() {

  cv::Mat loaded_imaes[num_images];

  for(int i = 0; i < num_images; i++) {
    std::string cur_image_path = std::string("../pi_src/media/test") + std::to_string(i+1) + std::string(".jpg");
    loaded_imaes[i] = cv::imread(cur_image_path);
  }
  int image_index = 0;


  do {
    cv::Mat bgr, warped, sobel_line, histogram;

    if (++image_index >= num_images)
      image_index = 0;

// ======== image processing pipeline ========

// load image
    bgr = loaded_imaes[image_index];
    cv::resize(bgr, bgr, cv::Size(1280, 720));
    cv::imshow("input image", bgr);

    //TODO: distortion correction

// generate binary:

    // perspective_warp
    perspective_warp(bgr, warped);

    // sobel filtering
    sobel_filtering(warped, sobel_line, 20, 255);
    cv::imshow("sobel_line", sobel_line);

    // (TODO more binary filtering)

    // histogram peak detection
    lane_histogram(sobel_line, histogram, sobel_line.rows/2);

// calculate lines:
    // TODO window search
    std::vector<WindowBox> left_boxes, right_boxes;
    window_search(sobel_line, histogram, left_boxes, right_boxes, 12, 200);

    draw_boxes(warped, left_boxes);
    draw_boxes(warped, right_boxes);
    cv::imshow("warped", warped);

    std::vector<double> lx;
    std::vector<double> ly;
    std::vector<double> coeffs;

    for(auto & box: left_boxes) {
      lx.push_back(box.get_center().x);
      ly.push_back(box.get_center().y);
    }

    PolynomialRegression<double> poly_regr;
    if(poly_regr.fitIt(lx, ly, 2, coeffs)) {
      std::cout << "equation: f=" << coeffs[0] << " + " << coeffs[1] << "x + " << coeffs[2]  << "x^2" << std::endl;
    }


    std::cout << "lbs " << left_boxes.size() << " rbs " << right_boxes.size() << std::endl;

    // TODO curve fitting

// output images:

    // TODO draw overlay

    // send/display video
    //cv::imshow("histogram", histogram);

// ========= autonomous driving ========

    // TODO calculate speed and steering

    // TODO request for obstacles/state - hande them

      // if no ground pause until ground available:
          // check n more times for ground then
          // continue;

    // TODO if no obstacles send speed and steering to arduino

  } while(cv::waitKey(0) != 'q');

  return 0;
}
