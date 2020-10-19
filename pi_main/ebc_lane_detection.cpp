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

int main() {

  CameraCapture cam(0);
  cam.set(cv::CAP_PROP_FPS, 30);			// Kamera Framerate auf 30 fps

  //VideoServer srv;				// Klasse für den VideoServer

  srv.namedWindow("input image");
	srv.namedWindow("sobel_line");
	srv.namedWindow("warped");
  srv.namedWindow("sobel_grad");
  srv.namedWindow("sobel_thresholded_img");
  srv.namedWindow("hls");
  srv.namedWindow("sobel_line");

  while(1) {
    cv::Mat bgr, warped, sobel_line, histogram;

// ======== image processing pipeline ========

// load image
    while(!cam.read(bgr)){}
    cv::resize(bgr, bgr, cv::Size(1000, 600));
    srv.imshow("input image", bgr);

    //TODO: distortion correction

// generate binary:

    // perspective_warp
    perspective_warp(bgr, warped);

    // sobel filtering
    sobel_filtering(warped, sobel_line, 20, 255);
    srv.imshow("sobel_line", sobel_line);

    // (IDEA more binary filtering)

    // histogram peak detection
    lane_histogram(sobel_line, histogram, sobel_line.rows/2);

// calculate lines:
    // window search
    std::vector<WindowBox> left_boxes, right_boxes;
    window_search(sobel_line, histogram, left_boxes, right_boxes, 12, 200);

    std::cout << "lbs " << left_boxes.size() << " rbs " << right_boxes.size() << std::endl;

    std::vector<cv::Point> midpoints;
    calc_midpoints(left_boxes, right_boxes, midpoints);

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

// output images:

    // TODO draw overlay

    draw_boxes(warped, left_boxes);
    draw_boxes(warped, right_boxes);
    srv.imshow("warped", warped);

    // send/display video
    //srv.imshow("histogram", histogram);

  }

  return 0;
}
