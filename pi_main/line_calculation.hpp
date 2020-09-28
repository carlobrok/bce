#ifndef LINE_CALCULATION
#define LINE_CALCULATION

#include <opencv2/opencv.hpp>


class WindowBox {

  cv::Point m_center;

  int m_width, m_height;
  bool m_lane_found;
  int m_min_count;

public:

  WindowBox();
  WindowBox(cv::Point p_start, int width, int height, int min_count = 50);

  bool has_lane() const { return m_lane_found; }
  void find_lane(cv::Mat &line_binary);

  int get_width() { return m_width;}
  int get_height() { return m_height;}
  cv::Point get_center() const { return m_center; }
  cv::Point get_next_start() { return cv::Point(m_center.x, m_center.y - m_height); }
  cv::Rect get_window_rect(cv::Mat &img) const;
};


void window_search(cv::Mat &warped, cv::Mat &histogram, std::vector<WindowBox>& left_boxes, std::vector<WindowBox>& right_boxes, int n_windows, int window_width);

void lane_peaks(cv::Mat const& histogram, cv::Point& left_max_loc, cv::Point& right_max_loc);

void find_lane_windows(cv::Mat& binary_img, WindowBox& window_box, std::vector<WindowBox>& wboxes);

void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order);

void draw_boxes(cv::Mat& img, const std::vector<WindowBox>& boxes);

void calc_midpoints(const std::vector<WindowBox>& left_boxes, const std::vector<WindowBox>& right_boxes, std::vector<cv::Point> & midpoints);

#endif
