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

  CameraCapture cam(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //cam.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  //cam.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  cam.set(cv::CAP_PROP_FPS, 30);			// Kamera Framerate auf 30 fps

  srv::init(true);				// Klasse für den VideoServer

  srv::namedWindow("input image");
	srv::namedWindow("warped");
  srv::namedWindow("sobel_grad");
  srv::namedWindow("sobel_thresh");
  srv::namedWindow("hls");
  srv::namedWindow("sobel_line");

  cv::Mat bgr, warped, sobel_line, histogram;
  while(1) {

    auto tstart = std::chrono::system_clock::now();

// ======== image processing pipeline ========

// load image

    while(!cam.read(bgr)){}
    //cv::resize(bgr, bgr, cv::Size(1000, 600));
    cv::GaussianBlur(bgr, bgr, cv::Size(5,5),2,2);		// Gaussian blur to normalize image
    srv::imshow("input image", bgr);

    auto timg_read = std::chrono::system_clock::now();
    std::cout << "image read: " << std::chrono::duration_cast<std::chrono::milliseconds>(timg_read - tstart).count() << "ms" << std::endl;
    //TODO: distortion correction

// generate binary:

    // perspective_warp
    perspective_warp(bgr, warped);
    auto tpersp_warp = std::chrono::system_clock::now();
    std::cout << "perspective_warp: " << std::chrono::duration_cast<std::chrono::milliseconds>(tpersp_warp - timg_read).count() << "ms" << std::endl;

    // sobel filtering
    sobel_filtering(warped, sobel_line, 20, 255);
    srv::imshow("sobel_line", sobel_line);
    auto tsobel = std::chrono::system_clock::now();
    std::cout << "sobel: " << std::chrono::duration_cast<std::chrono::milliseconds>(tsobel - tpersp_warp).count() << "ms" << std::endl;

    // (IDEA more binary filtering)

    // histogram peak detection
    lane_histogram(sobel_line, histogram, sobel_line.rows/2);
    auto tbin_img = std::chrono::system_clock::now();
    std::cout << "generate binary: " << std::chrono::duration_cast<std::chrono::milliseconds>(tbin_img - timg_read).count() << "ms" << std::endl;

// calculate lines:
    // window search
    std::vector<WindowBox> left_boxes, right_boxes;
    window_search(sobel_line, histogram, left_boxes, right_boxes, 12, 200);

    std::cout << "lbs " << left_boxes.size() << " rbs " << right_boxes.size() << std::endl;

    std::vector<cv::Point> midpoints;
    calc_midpoints(left_boxes, right_boxes, midpoints);

    auto tline_calc = std::chrono::system_clock::now();
    std::cout << "calculate lines: " << std::chrono::duration_cast<std::chrono::milliseconds>(tline_calc - tbin_img).count() << "ms" << std::endl;
// ========= autonomous driving ========

    // calculate speed from midpoints
    int speed = calc_speed(midpoints);

    // calculate steering
    double angle = calc_angle(warped, midpoints, true);

    std::cout << "speed, angle: " << speed << ", " << angle << std::endl;

    // TODO request for obstacles/state - hande them

      // if no ground pause until ground available:
          // check n more times for ground then
          // continue;

    // TODO if no obstacles send speed and steering to arduino

    std::cout << "send data: " << mot::set_dir_pwm_steer(mot::FORWARD, speed, angle) << std::endl;

// output images:

    // TODO draw overlay

    draw_boxes(warped, left_boxes);
    draw_boxes(warped, right_boxes);
    srv::imshow("warped", warped);

    // send/display video
    //srv::imshow("histogram", histogram);

    auto tend = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tend - tstart).count();
    std::cout << "whole proc: " << ms << "ms  / " << 1000 / ms << "fps" <<  std::endl;

    std::cout << "-------------------------------" << std::endl;
  }

  return 0;
}
