#include "img_processing.hpp"

#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

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


// input Mat: warped rgb image of lines
// output Mat: binary of all edges in the image (hopefully just lines)
void sobel_filtering(cv::Mat &warped, cv::Mat &sobel_line, int sobel_min, int sobel_max, int sobel_threshold) {
  cv::Mat hls, hls_split[3];
  cv::Mat sobel_x, sobel_y, sobel_grad, sobel_thresholded_img;
  cv::Mat combined;

  cv::cvtColor(warped, hls, cv::COLOR_BGR2HLS);
  cv::split(hls, hls_split);

  // perform thresholding on parallel
	std::thread x_direct(sobel_abs_thresh, std::ref(hls_split[2]), std::ref(sobel_x), 1, 0, sobel_min, sobel_max, 3);
	std::thread y_direct(sobel_abs_thresh, std::ref(hls_split[2]), std::ref(sobel_y), 0, 1, sobel_min, sobel_max, 3);
	x_direct.join();
	y_direct.join();

  addWeighted(sobel_x, 0.5, sobel_y, 0.5, 0, sobel_grad);

  cv::threshold(sobel_grad, sobel_thresholded_img, sobel_threshold, 255, cv::THRESH_BINARY);
  cv::morphologyEx(sobel_thresholded_img, sobel_line, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

  cv::imshow("hls", hls);
  cv::imshow("sobel_line", sobel_line);
}

void sobel_abs_thresh(const cv::Mat &src, cv::Mat &dst, int dx, int dy, int thresh_min, int thresh_max, int kernel_size) {
  int ddepth = CV_64F;
  cv::Mat grad_img, scaled;

  cv::Sobel(src, grad_img, ddepth, dx, dy, kernel_size);
  cv::convertScaleAbs(grad_img, scaled);
  cv::inRange(scaled, cv::Scalar(thresh_min), cv::Scalar(thresh_max), dst);
  return;
}

void lane_histogram(cv::Mat &line, cv::Mat &histogram) {
  // only use lower half of the image
  cv::Mat cropped = line(cv::Rect(0, line.rows / 2, line.cols, line.rows / 2));
  //cv::imshow("cropped", cropped);
  // generate histogram of the cropped image
  cv::reduce(cropped / 255, histogram, 0, cv::REDUCE_SUM, CV_32S);
  return;
}












// experimental

void edge_detection(cv::Mat &img_bgr, cv::Mat &edges, int gauss_kernel_size = 5) {
  cv::Mat gray, gauss_blur;
  cv::cvtColor(img_bgr, gray, cv::COLOR_BGR2GRAY);

  cv::GaussianBlur(gray, gauss_blur, cv::Size(gauss_kernel_size, gauss_kernel_size), 0);
  cv::Canny(gauss_blur, edges, 50,150);
}
