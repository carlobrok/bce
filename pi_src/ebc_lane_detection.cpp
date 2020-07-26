#include "devices.h"
#include "VideoServer.h"
#include "CameraCapture.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

const int num_images = 11;
//std::string images[num_images] = {};


void separate_lines(cv::Mat &warped, cv::Mat &line_binary, int sobel_threshold = 15, int hls_s_threshold = 100) {
  cv::Mat hls, hls_split[3], hls_s_t;
  cv::Mat sobel_x, grad_x, sobel_t;
  cv::Mat combined;

  cv::cvtColor(warped, hls, cv::COLOR_BGR2HLS);
  cv::split(hls, hls_split);

  cv::Sobel(hls_split[1], sobel_x, CV_64F, 1, 0);
  cv::convertScaleAbs(sobel_x, grad_x);

  cv::threshold(grad_x, sobel_t, sobel_threshold, 255, cv::THRESH_BINARY);
  cv::threshold(hls_split[2], hls_s_t, hls_s_threshold, 255, cv::THRESH_BINARY);
  cv::bitwise_or(sobel_t, hls_s_t, combined);

  cv::morphologyEx(combined, line_binary, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

  cv::imshow("hls", hls);
  cv::imshow("line_binary", line_binary);
}

void edge_detection(cv::Mat &img_bgr, cv::Mat &edges, int gauss_kernel_size = 5) {
  cv::Mat gray, gauss_blur;
  cv::cvtColor(img_bgr, gray, cv::COLOR_BGR2GRAY);

  cv::GaussianBlur(gray, gauss_blur, cv::Size(gauss_kernel_size, gauss_kernel_size), 0);
  cv::Canny(gauss_blur, edges, 50,150);
}

void perspective_warp(cv::Mat &input, cv::Mat &warped) {
  cv::Point2f src[4] = {{0.26,0.6},{0.73,0.6},{0,1},{1,1}};
  cv::Point2f dst[4] = {{0,0},{1,0},{0,1},{1,1}};

  for(auto &p : src) {
    p.x *= input.cols;
    p.y *= input.rows;
  }

  for(auto &p : dst) {
    p.x *= input.cols;
    p.y *= input.rows;
  }

  auto M = cv::getPerspectiveTransform(src, dst);

  cv::warpPerspective(input, warped, M, cv::Size(input.cols, input.rows));
}




int main() {

  cv::Mat loaded_imaes[num_images];

  for(int i = 0; i < num_images; i++) {
    std::string cur_image_path = std::string("../pi_src/media/test") + std::to_string(i+1) + std::string(".jpg");
    loaded_imaes[i] = cv::imread(cur_image_path);
  }
  int image_index = 0;


  do {
    cv::Mat bgr, filtered_lines, edges, warped, line;

    if (++image_index >= num_images)
      image_index = 0;

    bgr = loaded_imaes[image_index];
    cv::resize(bgr, bgr, cv::Size(1280, 720));
    cv::imshow("input image", bgr);

    perspective_warp(bgr, warped);
    separate_lines(warped, line, 50, 70);


    cv::imshow("warped", warped);

    //filter_line(bgr, filtered_lines, cv::Scalar::all(100));
    //cv::imshow("filtered_lines", filtered_lines);


    //edge_detection(filtered_lines, edges);

    //cv::imshow("edges", edges);

  } while(cv::waitKey(0) != 'q');

  return 0;
}
