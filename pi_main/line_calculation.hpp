#ifndef LINE_CALCULATION
#define LINE_CALCULATION

#include <opencv2/opencv.hpp>
#include <cmath>

class WindowBox {
private:
  cv::Point m_center, m_top_left, m_bottom_right;
  cv::Size m_window_size, m_image_size;

  bool m_lane_found;
  int m_min_count;

public:

  WindowBox();
  WindowBox(cv::Point p_start, cv::Size window_size, cv::Size image_size, int min_count = 50);

  bool has_lane() const { return m_lane_found; }
  void find_lane(cv::Mat &line_binary);

  cv::Size size() { return m_window_size; }
  cv::Point center() const { return m_center; }
  cv::Rect window_rect() const { return cv::Rect(m_top_left, m_bottom_right); }
 
  cv::Point next_p_start() { return m_center - cv::Point(0, m_window_size.height); }
  WindowBox next_box() { return WindowBox(next_p_start(), m_window_size, m_image_size); }
};

void window_search(cv::Mat &line_binary, cv::Mat &histogram, std::vector<WindowBox>& left_boxes, std::vector<WindowBox>& right_boxes, int n_windows, cv::Size window_size);

void lane_peaks(cv::Mat const& histogram, cv::Point& left_max_loc, cv::Point& right_max_loc);

void find_lane_windows(cv::Mat& binary_img, WindowBox &window_box, std::vector<WindowBox>& wboxes, int n_windows);

void draw_boxes(cv::Mat& img, const std::vector<WindowBox>& boxes);

void boxes_to_line(std::vector<WindowBox>& boxes, cv::Vec4f & line);



class lane_line {
private:

  cv::Vec4f m_line;

public:

  lane_line(cv::Vec4f line) : m_line(line) {};

  bool has_lane() { return (cv::countNonZero(m_line) > 0); }

  cv::Vec4f line() { return m_line; }
  cv::Point top (int img_height);
  cv::Point bottom (int img_height);
  
  void draw (cv::Mat & img);

  float angle() { return atan( - m_line[0] / m_line[1] ) * 180 / 3.142; }
};

lane_line calc_midline(lane_line left, lane_line right, cv::Size image_size);

// obsolete
void calc_midpoints(const std::vector<WindowBox>& left_boxes, const std::vector<WindowBox>& right_boxes, std::vector<cv::Point> & midpoints);

#endif
