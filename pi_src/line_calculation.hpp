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



/*

class old_WindowBox {
private:

  int x_left, x_center, x_right;
  int y_bottom, y_top;
  int width, height, mincount;
  bool lane_found;
  cv::Mat img_window;
  std::vector<cv::Point> nonzero;

  int count_nonzero(void) const { return nonzero.size(); }
  bool is_noise(void) const { return (count_nonzero() > img_window.rows * img_window.cols * .75); }

public:

  WindowBox() : x_left(0), x_center(0), x_right(0),
    y_bottom(0), y_top(0),
    width(0), height(0),
    mincount(0), lane_found(false) {};

  WindowBox(cv::Mat& binary_img, int x_center, int y_top,
    int width = 220, int height = 80,
    int mincount = 50, bool lane_found = false);

  inline friend std::ostream& operator<< (std::ostream& out, WindowBox const& window);
  friend void find_lane_windows(cv::Mat& binary_img, WindowBox& window_box, std::vector<WindowBox>& wboxes);


  // getters
  void get_centers(int& x_center, int& y_center) const { x_center = this->x_center; y_center = (y_top - y_bottom) / 2; }
  void get_indices(cv::Mat& x, cv::Mat& y) const;
  const cv::Point get_bottom_left_point(void) const { return cv::Point(x_left, y_bottom); }
  const cv::Point get_top_right_point(void) const { return cv::Point(x_right, y_top); }
  const WindowBox get_next_windowbox(cv::Mat& binary_img) const;

  // hassers
  bool has_line(void) const { return ((count_nonzero() > mincount) || is_noise()); }
  bool has_lane(void);

};*/

void window_search(cv::Mat &warped, cv::Mat &histogram, std::vector<WindowBox>& left_boxes, std::vector<WindowBox>& right_boxes, int n_windows, int window_width);

void lane_peaks(cv::Mat const& histogram, cv::Point& left_max_loc, cv::Point& right_max_loc);

void find_lane_windows(cv::Mat& binary_img, WindowBox& window_box, std::vector<WindowBox>& wboxes);

void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order);

void draw_boxes(cv::Mat& img, const std::vector<WindowBox>& boxes);

#endif
