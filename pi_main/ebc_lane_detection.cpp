#include "devices.hpp"
#include "img_processing.hpp"
#include "line_calculation.hpp"
#include "drive_calculation.hpp"
#include "VideoServer.h"
#include "CameraCapture.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>

#include <chrono>
#include <thread>

int main() {

  srv::init(true);				// init VideoServer

  CameraCapture cam(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //cam.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  //cam.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  cam.set(cv::CAP_PROP_FPS, 30);			// Kamera Framerate auf 30 fps


  srv::namedWindow("input image");
  //srv::namedWindow("warped");
  srv::namedWindow("binary_line");
  // only for color filtering
  srv::namedWindow("hsv");
  srv::namedWindow("color_thresh");
  // only for sobel filtering
  /*srv::namedWindow("hls");
  srv::namedWindow("sobel_grad");
  srv::namedWindow("sobel_thresh");*/

  init_arduino(0x08);

  cv::Mat bgr, /*warped,*/ binary_line, histogram;

  //while(!cam.read(bgr)){}
  //cv::Mat transform_M = transform_matrix(bgr.size());

  while(1) {

    auto tstart = std::chrono::system_clock::now();

// ======== image processing pipeline ========

// load image

    while(!cam.read(bgr)){}
    //cv::resize(bgr, bgr, cv::Size(1000, 600));
    cv::GaussianBlur(bgr, bgr, cv::Size(5,5),2,2);		// Gaussian blur to normalize image

    auto timg_read = std::chrono::system_clock::now();
    std::cout << "image read: " << std::chrono::duration_cast<std::chrono::milliseconds>(timg_read - tstart).count() << "ms" << std::endl;
    //TODO: distortion correction

// generate binary:

    /*// perspective_warp
    perspective_warp(bgr, warped, transform_M);
    auto tpersp_warp = std::chrono::system_clock::now();
    std::cout << "perspective_warp: " << std::chrono::duration_cast<std::chrono::milliseconds>(tpersp_warp - timg_read).count() << "ms" << std::endl;
    */

    // color filtering
    color_filtering(bgr, binary_line, cv::Scalar(115,115,105), cv::Scalar(205, 220, 200));
    srv::imshow("binary_line", binary_line);
    auto tcolor = std::chrono::system_clock::now();
    std::cout << "color filtering: " << std::chrono::duration_cast<std::chrono::milliseconds>(tcolor - timg_read).count() << "ms" << std::endl;

    /*
    // sobel filtering
    sobel_filtering(warped, binary_line, 20, 255);
    srv::imshow("binary_line", binary_line);
    auto tsobel = std::chrono::system_clock::now();
    std::cout << "sobel: " << std::chrono::duration_cast<std::chrono::milliseconds>(tsobel - tpersp_warp).count() << "ms" << std::endl;
    */

    // (IDEA more binary filtering)

    // histogram peak detection
    lane_histogram(binary_line, histogram, binary_line.rows/2);
    auto tbin_img = std::chrono::system_clock::now();
    std::cout << "generate binary: " << std::chrono::duration_cast<std::chrono::milliseconds>(tbin_img - tcolor).count() << "ms" << std::endl;

// calculate lines:
    // window search
    std::vector<WindowBox> left_boxes, right_boxes;
    cv::Vec4f line_left, line_right;
    window_search(binary_line, histogram, left_boxes, right_boxes, 20, 200);

    std::cout << "lbs " << left_boxes.size() << " rbs " << right_boxes.size() << std::endl;

    boxes_to_line(left_boxes, line_left);
    boxes_to_line(right_boxes, line_right);

    std::cout << "line_left: " << line_left << " line_right" << line_right << std::endl;

    draw_line(bgr, line_left);
    draw_line(bgr, line_right);

    /*calc_midline(left_boxes, right_boxes);

    std::vector<cv::Point> midpoints;
    midpoints.reserve(12);
    calc_midpoints(left_boxes, right_boxes, midpoints);
    std::cout << "midpoints: " << midpoints.size() << std::endl;*/
    
    auto tline_calc = std::chrono::system_clock::now();
    std::cout << "calculate lines: " << std::chrono::duration_cast<std::chrono::milliseconds>(tline_calc - tbin_img).count() << "ms" << std::endl;

// ========= autonomous driving ========

    // calculate speed from midpoints
    /*int speed = calc_speed(midpoints);
    std::cout << speed << std::endl;

    // calculate steering
    double angle = calc_angle(bgr, midpoints, true);

    std::cout << "speed, angle: " << speed << ", " << angle << std::endl;

    // TODO request for obstacles/state - hande them

      // if no ground pause until ground available:
          // check n more times for ground then
          // continue;

    // TODO if no obstacles send speed and steering to arduino

    int e = mot::set_dir_pwm_steer(mot::FORWARD, speed, angle);
    if(e < 0) {
      std::cout << "Error sending i2c data: " << strerror(errno) << std::endl;
    } else {
      std::cout << "Successfully sent i2c data" << std::endl;
    }

    */

// output images:

    // TODO draw overlay

    draw_boxes(bgr, left_boxes);
    draw_boxes(bgr, right_boxes);
    //srv::imshow("warped", warped);
    srv::imshow("input image", bgr);

    // send/display video
    //srv::imshow("histogram", histogram);

    auto tend = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tend - tstart).count();
    std::cout << "whole proc: " << ms << "ms  / " << 1000 / ms << "fps" <<  std::endl;

    std::cout << "-------------------------------" << std::endl;
  }

  return 0;
}
