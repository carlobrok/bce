#include "line_calculation.hpp"

#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>
#include <cmath>

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


void roi_search(cv::Mat & binary_line, lane_data & lane_mid, lane_data & lane_left, lane_data & lane_right, int roi_height, int n_rois, int min_area_size) {
	
	std::vector<cv::Point> points_left, points_right;
	
	// bild in ROIs einteilen, entlang der midlane
    // alle ROIs links/rechts durchgehen -> funktion 
	for (int i_roi = 0; i_roi < n_rois; i_roi++)
	{
		cv::Point p_mid = lane_mid.at_y( i_roi * roi_height + (roi_height / 2) );
		
		// rois berechnen
		cv::Rect rect_roi_left(0, (i_roi + 1) * roi_height, p_mid.x, roi_height);
		cv::Rect rect_roi_right(p_mid.x, (i_roi + 1) * roi_height, binary_line.cols - p_mid.x, roi_height);

		// mat vom roi bereich erstellen
		cv::Mat roi_left = binary_line(rect_roi_left);
		cv::Mat roi_right = binary_line(rect_roi_right);
		
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
			points_right.push_back( cv::Point(p_mid.x + m_right.m10 / m_right.m00, p_mid.y));
		}
	}

	lane_left.set_data(points_left);
	lane_right.set_data(points_right);
}


lane_data calc_midline(lane_data & left, lane_data & right, cv::Size image_size) {
	cv::Point2f bot_mid((left.bottom().x + right.bottom().x) / 2, image_size.height);
	cv::Point2f top_mid((left.top().x + right.top().x) / 2, 0);
	cv::Vec2f vec(bot_mid.x - top_mid.x, bot_mid.y - top_mid.y);

	// TODO: funktioniert nur, wenn zwei virhanden sind.
	return lane_data(vec, bot_mid,'m');
}