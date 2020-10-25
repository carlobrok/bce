#include "calc_poly.hpp"
#include "opencv2/opencv.hpp"

void draw_poly(int event, int x, int y, int flags, void* userdata) {
  // IDEA: remove nearest point to mouse within 50px radius when right button down

  if(event == cv::EVENT_LBUTTONDOWN) {
    warp_poly_points.push_back(cv::Point(x,y));
  }
}

void calc_transform(cv::Mat & input) {

  cv::namedWindow("draw_poly");
  cv::namedWindow("preview");
  cv::Mat preview;

  cv::imshow("draw_poly", input);
  cv::setMouseCallback("draw_poly", draw_poly, NULL);
  bool complete = false;

  std::cout << "draw polygon, press enter to complete" << std::endl;

  do {

    cv::polylines(input, warp_poly_points, true, cv::Scalar(64, 157, 245));

    if(warp_poly_points.size() == 4) {
      std::vector<cv::Point2f> dst{{0,0},{1,0},{0,1},{1,1}};

      /*for(auto &p : warp_poly_points) {
        p.x *= input.cols;
        p.y *= input.rows;
      }*/

      for(auto &p : dst) {
        p.x *= input.cols;
        p.y *= input.rows;
      }

      auto M = cv::getPerspectiveTransform(warp_poly_points, dst);

      cv::warpPerspective(input, preview, M, cv::Size(input.cols, input.rows), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
      cv::imshow("preview", preview);
    }

    auto k = cv::waitKey(100);
    if (k == '\n') complete = true;

  } while(!complete);
  cv::destroyWindow("preview");
  cv::destroyWindow("draw_poly");
}
