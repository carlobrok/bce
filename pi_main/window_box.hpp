#ifndef WINDOW_BOX
#define WINDOW_BOX

#include <opencv2/opencv.hpp>

class WindowBox {
private:
	cv::Point m_center, m_top_left, m_bottom_right;
	cv::Size m_window_size, m_image_size;

	bool m_lane_found;
	int m_min_count;

public:

	WindowBox();
	WindowBox(cv::Point p_start, cv::Size window_size, cv::Size image_size, int min_count = 50);

	bool lane_found() const { return m_lane_found; }
	void find_lane(cv::Mat &line_binary);

	cv::Size size() { return m_window_size; }
	cv::Point center() const { return m_center; }
	cv::Rect window_rect() const { return cv::Rect(m_top_left, m_bottom_right); }
	
	cv::Point next_p_start() { return m_center - cv::Point(0, m_window_size.height); }
	WindowBox next_box() { return WindowBox(next_p_start(), m_window_size, m_image_size); }
};

#endif