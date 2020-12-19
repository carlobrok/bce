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

void window_search(cv::Mat &line_binary, cv::Mat & histogram, std::vector<WindowBox>& boxes_left, std::vector<WindowBox>& boxes_right, int n_windows, cv::Size window_size) {

	CV_Assert( window_size.height * n_windows <= line_binary.rows );

	cv::Point peak_left, peak_right;
	lane_peaks(histogram, peak_left, peak_right); // Peaks

	peak_left.y = line_binary.rows - window_size.height/2;
	peak_right.y = line_binary.rows - window_size.height/2;

	//std::cout << "starting at: l" << peak_left << "  r" << peak_right << std::endl;

	// Initialise left and right window boxes
	WindowBox wbl(peak_left,  window_size, line_binary.size());
	WindowBox wbr(peak_right, window_size, line_binary.size());

	// Parallelize searching
	std::thread left(find_lane_windows, std::ref(line_binary), std::ref(wbl), std::ref(boxes_left), n_windows);
	std::thread right(find_lane_windows, std::ref(line_binary), std::ref(wbr), std::ref(boxes_right), n_windows);
	left.join();
	right.join();

}

void find_lane_windows(cv::Mat& binary_img, WindowBox &window_box, std::vector<WindowBox>& wboxes, int n_windows)
{
	for ( int n = 0; n < n_windows && window_box.center().y > 0; n++) {
		window_box.find_lane(binary_img);
		wboxes.push_back(window_box);
		window_box = window_box.next_box();
		//std::cout << "next start: " << window_box.center() << std::endl;
	}

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
		if(box.lane_found()) {
			cv::rectangle(img, box.window_rect(), cv::Scalar(0, 255, 0), 2);
			cv::circle(img, box.center(), 2, cv::Scalar(0,0,255), 2);
		}
	}
}


void roi_search(cv::Mat & binary_line, lane_data & lane_mid, lane_data & lane_left, lane_data & lane_right, int roi_height, int n_rois, int min_area_size = 30) {
	
	std::vector<cv::Point> points_left, points_right;
	
	// bild in ROIs einteilen, entlang der midlane
    // alle ROIs links/rechts durchgehen -> funktion 
	for (int i_roi = 0; i_roi < n_rois; i_roi++)
	{
		cv::Point p_mid = lane_mid.at_y( i_roi * roi_height + (roi_height / 2) );
		
		cv::Mat roi_left = binary_line(cv::Rect(0, i_roi * roi_height, p_mid.x, roi_height));
		cv::Mat roi_right = binary_line(cv::Rect(p_mid.x, i_roi * roi_height, binary_line.cols - p_mid.x, roi_height));
		
		// moments der ROIs bestimmen

		cv::Moments m_left = cv::moments(roi_left,true);
		cv::Moments m_right = cv::moments(roi_right,true);

		// moments area größer als min wert?
		if(m_left.m00 > min_area_size) {
			// Mittelpunkt der Fläche bestimmen
			// Mittelpunkt in vector für neue linie links/rechts schreiben
			points_left.push_back( cv::Point(m_left.m10 / m_left.m00, p_mid.y));
		}
		if(m_right.m00 > min_area_size) {
			// Mittelpunkt der Fläche bestimmen
			// Mittelpunkt in vector für neue linie links/rechts schreiben
			points_right.push_back( cv::Point(m_right.m10 / m_right.m00, p_mid.y));
		}
	}

	lane_left.set_data(points_left);
	lane_right.set_data(points_right);
}


void lane_data::set_data(std::vector<cv::Point>& center_points, int min_points, bool reset) {

	// minimum of 3 points are required
	if (center_points.size() > min_points) {
		cv::Vec4f line_vec;
		cv::fitLine(center_points, line_vec, cv::DIST_L2, 0, 0.01, 0.01);
		vec = cv::Vec2f(line_vec[0], line_vec[1]);
		point = cv::Point(line_vec[2], line_vec[3]);
	} 
	// if less than 3 points reset vec and point if reset=true
	else if(reset) {
		vec = cv::Vec2f(0,0);
		point = cv::Point(0,0);
	}
}

void lane_data::set_data(std::vector<WindowBox>& boxes, int min_points, bool reset) {
	std::vector<cv::Point> points;

	std::cout << "set line_data by WindowBoxes: ";
	for(WindowBox & b : boxes) {
		if(b.lane_found()) {
			points.push_back(b.center());
			std::cout << b.center() << ", ";
		}
	}
	std::cout << std::endl;

	set_data(points, min_points, reset);
}


lane_data calc_midline(lane_data & left, lane_data & right, cv::Size image_size) {
	cv::Point2f bot_mid((left.bottom().x + right.bottom().x) / 2, image_size.height);
	cv::Point2f top_mid((left.top().x + right.top().x) / 2, 0);
	cv::Vec2f vec(bot_mid.x - top_mid.x, bot_mid.y - top_mid.y);

	// TODO: funktioniert nur, wenn zwei virhanden sind.
	return lane_data(vec, bot_mid,'m');
}

// lane line ==========================

void lane_data::draw(cv::Mat & img, cv::Scalar color) {
	if(has_lane()) {
		std::cout << "p_top " << top() << " / p_bottom " << bottom() << std::endl;
		cv::line(img, top(), bottom(), color);
	}
}

std::ostream& operator<<(std::ostream& os, const lane_data& ll) { 
    return os << "[" << ll.id << ": v" << ll.vec << ", p" << ll.point << "]"; 
} 

/*
 * calculation for x values:
 * (x,y) = t*(vx,vy) + (x0,y0) 
 * if y is given:  =>  x = (y - y0) / vy * vx + x0
*/

cv::Point lane_data::top() {
	if(!has_lane()) return cv::Point(0,0);
	return cv::Point((-point.y) / vec[1] * vec[0] + point.x, 0);	// calculate x value with y = 0;
}

cv::Point lane_data::at_y(int y) {
	if(!has_lane()) return cv::Point(0,y);
	return cv::Point((y - point.y) / vec[1] * vec[0] + point.x, y);	// calculate x value with y = image_height
}

cv::Point lane_data::bottom() {
	if(!has_lane()) return cv::Point(0,image_height);
	return cv::Point((image_height - point.y) / vec[1] * vec[0] + point.x, image_height);	// calculate x value with y = image_height
}

double lane_data::distance(lane_data & line2, double angle_factor) {
	return (bottom().x - line2.bottom().x) * (bottom().x - line2.bottom().x) + angle_factor * (angle() - line2.angle()) * (angle() - line2.angle()); 
}