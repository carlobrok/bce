#include "calc_poly.hpp"
#include "opencv2/opencv.hpp"

#include <vector>
#include <algorithm>

sort_x comp_x;
sort_y comp_y;

void draw_poly(int event, int x, int y, int flags, void* userdata) {
  // IDEA: remove nearest point to mouse within 50px radius when right button down

  if(event == cv::EVENT_MOUSEMOVE && flags == cv::EVENT_FLAG_LBUTTON && warp_poly_points.size() == 4) {
      // search for nearest point
      int idx_near = -1;
      double d_near = INFINITY;
      for (int i = 0; i < 4; i++) {
        auto p = warp_poly_points[i];
        auto d = sqrt((p.x-x)*(p.x-x) + (p.y-y)*(p.y-y));
        if(d < d_near) {
          idx_near = i;
          d_near = d;
        }
      }
      CV_Assert(idx_near != -1);
      // replace nearest point with cursor x/y
      warp_poly_points[idx_near] = cv::Point(x,y);
  }
  if(event == cv::EVENT_LBUTTONDOWN && warp_poly_points.size() < 4) {
    warp_poly_points.push_back(cv::Point(x,y));
  }
}

void calc_transform(cv::Mat & input) {

  cv::Mat preview;
  cv::Mat draw_image = input.clone();
  
  warp_poly_points.clear();
  std::vector<cv::Point> src_points;

  cv::setMouseCallback("input image", draw_poly, NULL);
  bool complete = false;

  std::cout << "draw polygon, press enter to complete" << std::endl;

  do {
    draw_image = input.clone();
    cv::polylines(draw_image, warp_poly_points, true, cv::Scalar(64, 157, 245));
    cv::imshow("input image", draw_image);

    if(warp_poly_points.size() == 4) {
      cv::Point2f src[4];     // warp_poly_points vector to array
      cv::Point2f dst[4] = {{0,0},{1,0},{0,1},{1,1}};
      
      src_points = warp_poly_points;

      std::sort(src_points.begin(), src_points.end(), comp_y);
      std::sort(src_points.begin(), src_points.begin()+2, comp_x);
      std::sort(src_points.begin()+2, src_points.end(), comp_x);  
      
      //std::cout << "sorted: " << src_points << std::endl; 

      std::copy(src_points.begin(), src_points.end(), src);

      /*for(auto &p : warp_poly_points) {
        p.x *= input.cols;
        p.y *= input.rows;
      }*/

      for(auto &p : dst) {
        p.x *= input.cols;
        p.y *= input.rows;
      }

      auto M = cv::getPerspectiveTransform(src, dst);

      cv::warpPerspective(input, preview, M, cv::Size(input.cols, input.rows), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
      cv::imshow("preview", preview);
    }

    int k = cv::waitKey(1);
    if (k == 13) {
      complete = true;
    }
  } while(!complete);
  cv::destroyWindow("preview");
  
}
