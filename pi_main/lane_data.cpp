#include "lane_data.hpp"
#include "window_box.hpp"

#include <opencv2/opencv.hpp>
#include <ostream>

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