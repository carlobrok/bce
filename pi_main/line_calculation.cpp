#include "line_calculation.hpp"

#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>
#include <cmath>


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

void window_search(cv::Mat &line_binary, cv::Mat &histogram, std::vector<WindowBox>& left_boxes, std::vector<WindowBox>& right_boxes, int n_windows, int window_width) {

	cv::Point peak_left, peak_right;
	lane_peaks(histogram, peak_left, peak_right); // Peaks

	int window_height = line_binary.rows / n_windows;

	peak_left.y = line_binary.rows - window_height/2;
	peak_right.y = line_binary.rows - window_height/2;

	//std::cout << "starting at: l" << peak_left << "  r" << peak_right << std::endl;

	// Initialise left and right window boxes
	WindowBox wbl(peak_left,  cv::Size(window_width, window_height), line_binary.size());
	WindowBox wbr(peak_right, cv::Size(window_width, window_height), line_binary.size());

	// Parallelize searching
	std::thread left(find_lane_windows, std::ref(line_binary), std::ref(wbl), std::ref(left_boxes));
	std::thread right(find_lane_windows, std::ref(line_binary), std::ref(wbr), std::ref(right_boxes));
	left.join();
	right.join();

}

void find_lane_windows(cv::Mat& binary_img, WindowBox &window_box, std::vector<WindowBox>& wboxes)
{
	do {
		window_box.find_lane(binary_img);
		wboxes.push_back(window_box);
		window_box = window_box.next_box();
		//std::cout << "next start: " << window_box.center() << std::endl;
	} while (window_box.center().y > 0);

	return;
}

void lane_peaks(cv::Mat const& histogram, cv::Point& left_max_loc, cv::Point& right_max_loc)
{
	// TODO: find a method to handle shadows
	cv::Point temp;
	double min, max;
	int midpoint = histogram.cols / 2;

	cv::Mat left_half = histogram.colRange(0, midpoint);
	cv::Mat right_half = histogram.colRange(midpoint, histogram.cols);

	cv::minMaxLoc(left_half, &min, &max, &temp, &left_max_loc);
	cv::minMaxLoc(right_half, &min, &max, &temp, &right_max_loc);
	right_max_loc = right_max_loc + cv::Point(midpoint, 0);

	return;
}

void draw_boxes(cv::Mat& img, const std::vector<WindowBox>& boxes) {
	// Draw the windows on the output image
	for (const auto& box : boxes) {
		if(box.has_lane()) {
			cv::rectangle(img, box.window_rect(), cv::Scalar(0, 255, 0), 2);
			cv::circle(img, box.center(), 2, cv::Scalar(0,0,255), 2);
		}
	}
}

void boxes_to_line(std::vector<WindowBox>& boxes, cv::Vec4f & line) {
	std::vector<cv::Point> points;

	std::cout << "points: ";
	for(WindowBox & b : boxes) {
		if(b.has_lane()) {
			points.push_back(b.center());
			std::cout << b.center() << ", ";
		}
	}
	std::cout << std::endl;
	if (points.size() > 0) {
		cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
	} else {
		line.zeros();
	}
}


// lane line ==========================

void lane_line::draw(cv::Mat & img) {
	if(has_lane()) {
		std::cout << "p_top " << top(img.rows) << " / p_bottom " << bottom(img.rows) << std::endl;
		cv::line(img, top(img.rows), bottom(img.rows), cv::Scalar(0,0,255));
	}
}

// calculation for x values:
	// (x,y) = t*(vx,vy) + (x0,y0) 
	// if y is given:  =>  x = (y - y0) / vy * vx + x0
cv::Point lane_line::top(int img_height) {
	if(!has_lane()) return cv::Point(0,0);
	return cv::Point((-m_line[3]) / m_line[1] * m_line[0] + m_line[2], 0);	// calculate x value with y = 0;
}
cv::Point lane_line::bottom(int img_height) {
	if(!has_lane()) return cv::Point(0,img_height);
	return cv::Point((img_height - m_line[3]) / m_line[1] * m_line[0] + m_line[2], img_height);	// calculate x value with y = img_height
}


lane_line calc_midline(lane_line left, lane_line right, cv::Size image_size) {
	cv::Vec4f mid_line;

	if(left.has_lane() && right.has_lane()) {
		mid_line = left.line() + right.line();
	} else if (left.has_lane()) {
		mid_line = left.line();
	} else if (right.has_lane()) {
		mid_line = left.line();
	}
	/*cv::Point top_mid = (left.top(image_size.height) + right.top(image_size.height)) / 2;
	
	} else {
		mid_line = left.line() + right.line();	// add vectors to calculate: m0 = l0 + r0 and m1 = l1 + r1
	}*/

	// TODO: funktioniert nur, wenn zwei virhanden sind.
	mid_line[2] = (left.bottom(image_size.height).x + right.bottom(image_size.height).x) / 2;		// set point x on line
	mid_line[3] = image_size.height;		// set point y on line
	return lane_line(mid_line);
}