#include "window_box.hpp"

#include <opencv2/opencv.hpp>

// window box ================================0

WindowBox::WindowBox(cv::Point p_start, cv::Size window_size, cv::Size image_size, int min_count) {

	m_center = p_start;
	m_min_count = min_count;
	m_window_size = window_size;
	m_image_size = image_size;

	m_lane_found = false;
	
	m_top_left.x = m_center.x - window_size.width / 2;
	m_top_left.y = m_center.y - window_size.height / 2;

	m_bottom_right.x = m_center.x + window_size.width / 2;
	m_bottom_right.y = m_center.y + window_size.height / 2;

	if(m_top_left.x < 0) m_top_left.x = 0;
	else if(m_top_left.x > image_size.width) m_top_left.x = image_size.width;
	if(m_bottom_right.x > image_size.width) m_bottom_right.x = image_size.width;
}

void WindowBox::find_lane(cv::Mat &line_binary) {
	//std::cout << window_rect(line_binary) << std::endl;
	cv::Mat window_roi = line_binary(window_rect());

	cv::Moments w_moments = cv::moments(window_roi);
	if (isnan(w_moments.m10/w_moments.m00)) {
		m_lane_found = false;
		return;

	}
	//std::cout << w_moments.m10 / w_moments.m00 + m_center.x - m_width / 2 << std::endl;
	m_center.x = (int)(w_moments.m10 / w_moments.m00 + m_top_left.x);
	m_lane_found = true;
}