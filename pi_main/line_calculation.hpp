#ifndef LINE_CALCULATION
#define LINE_CALCULATION

#include <opencv2/opencv.hpp>


class WindowBox {
private:
  cv::Point m_center, m_top_left, m_bottom_right;
  cv::Size m_window_size;
  bool m_lane_found;
  int m_min_count;

public:

  WindowBox();
  WindowBox(cv::Point p_start, cv::Size window_size, cv::Size image_size, int min_count = 50);

  bool has_lane() const { return m_lane_found; }
  void find_lane(cv::Mat &line_binary);

  int width() { return m_window_size.width;}
  int height() { return m_window_size.height;}
  cv::Point center() const { return m_center; }
  cv::Point next_p_start() { return m_center - cv::Point(0, m_window_size.height); }
  cv::Rect window_rect() const;
};


void window_search(cv::Mat &warped, cv::Mat &histogram, std::vector<WindowBox>& left_boxes, std::vector<WindowBox>& right_boxes, int n_windows, int window_width);

void lane_peaks(cv::Mat const& histogram, cv::Point& left_max_loc, cv::Point& right_max_loc);

void find_lane_windows(cv::Mat& binary_img, WindowBox& window_box, std::vector<WindowBox>& wboxes);

void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order);

void draw_boxes(cv::Mat& img, const std::vector<WindowBox>& boxes);

void calc_midpoints(const std::vector<WindowBox>& left_boxes, const std::vector<WindowBox>& right_boxes, std::vector<cv::Point> & midpoints);

void draw_line(cv::Mat & img, cv::Vec4f & line);

void boxes_to_line(std::vector<WindowBox>& boxes, cv::Vec4f & line);

#endif
