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

#include <csignal>

const int n_windows = 12;

void signalHandler( int signum ) {
    std::cout << "Signal (" << signum << ") received: stopping car" << std::endl;

    mot::set_state(mot::OFF, true);
    // cleanup and close up stuff here  
    // terminate program  

    exit(signum);  
}

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

  // hande interrupt and segmentation fault to brake motor
  signal(SIGINT, signalHandler);
  signal(SIGSEGV, signalHandler);

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
    color_filtering(bgr, binary_line, cv::Scalar(115,53,105), cv::Scalar(205, 220, 205));
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
    // Vektor, der die Gerade/Linie beinhaltet
    cv::Vec4f line_left, line_right;
    window_search(binary_line, histogram, left_boxes, right_boxes, n_windows, cv::Size(200, binary_line.rows / 20));

    std::cout << "lbs " << left_boxes.size() << " rbs " << right_boxes.size() << std::endl;

    boxes_to_line(left_boxes, line_left);
    boxes_to_line(right_boxes, line_right);

    std::cout << "line_left: " << line_left << " line_right" << line_right << std::endl;

    lane_line lane_left(line_left);
    lane_line lane_right(line_right);
    lane_line lane_mid = calc_midline(lane_left, lane_right, bgr.size());

    lane_right.draw(bgr);
    lane_left.draw(bgr);
    lane_mid.draw(bgr);

    std::cout << "lane mid vec: "  << lane_mid.line() << std::endl; 

    auto tline_calc = std::chrono::system_clock::now();
    std::cout << "calculate lines: " << std::chrono::duration_cast<std::chrono::milliseconds>(tline_calc - tbin_img).count() << "ms" << std::endl;

// ========= autonomous driving ========

    // calculate speed from midpoints
    // int speed = calc_speed(midpoints);
    int speed = 50;

    // calculate steering
    double angle = lane_mid.angle();

    std::cout << "speed, angle: " << speed << ", " << angle << std::endl;

    // TODO request for obstacles/state - hande them

      // if no ground pause until ground available:
          // check n more times for ground then
          // continue;

    // TODO if no obstacles send speed and steering to arduino

    int e = mot::set_dir_pwm_steer(mot::FORWARD, speed, (int)angle);
    if(e < 0) {
      std::cout << "Error sending i2c data: " << strerror(errno) << std::endl;
    } else {
      std::cout << "Successfully sent i2c data" << std::endl;
    }

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
