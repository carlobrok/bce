#include "line_calculation.hpp"

#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>
#include <cmath>

WindowBox::WindowBox(cv::Point p_start, int width, int height, int min_count) {

	m_center = p_start;
	m_width = width;
	m_height = height;
	m_min_count = min_count;

	m_lane_found = false;

}

void WindowBox::find_lane(cv::Mat &line_binary) {
	//std::cout << get_window_rect(line_binary) << std::endl;
	cv::Mat window_roi = line_binary(get_window_rect(line_binary));

	cv::Moments w_moments = cv::moments(window_roi);
	if (isnan(w_moments.m10/w_moments.m00)) {
		m_lane_found = false;
		return;

	}
	//std::cout << w_moments.m10 / w_moments.m00 + m_center.x - m_width / 2 << std::endl;
	m_center.x = (int)(w_moments.m10 / w_moments.m00 + m_center.x - m_width / 2);
	m_lane_found = true;
}

cv::Rect WindowBox::get_window_rect(cv::Mat &img) const {
	cv::Rect rect(m_center.x - m_width/2, m_center.y - m_height/2, m_width, m_height);
	if(rect.x < 0) rect.x = 0;
	else if(rect.x > img.cols) rect.x = img.cols;
	if(rect.x + rect.width > img.cols) rect.width = img.cols - rect.x;

	return rect;
}

/*
WindowBox::WindowBox(cv::Mat& binary_img, int x_center, int y_top, int width, int height, int mincount, bool lane_found)
{
	this->x_center = x_center;
	this->y_top = y_top;
	this->width = width;
	this->height = height;
	this->mincount = mincount;
	this->lane_found = lane_found;

	// derived
	// identify window boundaries in x and y
	int margin = this->width / 2;
	x_left = this->x_center - margin;
	x_right = this->x_center + margin;
	y_bottom = y_top - this->height;

	// Identify the nonzero pixels in x and y within the window
	cv::Rect rect(cv::Point(x_left, y_bottom), cv::Point(x_right, y_top));
	cv::Mat img_window = binary_img(rect);
	cv::findNonZero(img_window, nonzero);
}

void WindowBox::get_indices(cv::Mat& x, cv::Mat& y) const
{
	// clear matrices
	x.release();
	y.release();

	int npoints = count_nonzero();
	x = cv::Mat::zeros(npoints, 1, CV_32F);
	y = cv::Mat::zeros(npoints, 1, CV_32F);

	for (int i = 0; i < npoints; i++) {
		x.at<float>(i, 0) = nonzero[i].x + x_left;
		y.at<float>(i, 0) = nonzero[i].y + y_bottom;
	}

	return;
}

const WindowBox WindowBox::get_next_windowbox(cv::Mat& binary_img) const
{
	if (y_bottom <= 0) return WindowBox(); // return empty box

	int new_y_top = y_bottom; // next box top starts at lasts bottom
	int new_x_center = x_center; // use existing center

	if (has_line()) {

		//cv::Moments m = cv::moments(nonzero, true);
		//new_x_center = m.m10/m.m00;

		double sum = 0;
		for (auto const& point : nonzero) {
			sum += (point.x + x_left);
		}
		new_x_center = sum / nonzero.size(); // recenter based on mean
		std::cout << "nonzeros: " << count_nonzero() << std::endl;
	}
	if (new_x_center + this->width / 2 > binary_img.cols) return WindowBox(); // if outside of ROI return empty box

	return WindowBox(binary_img, new_x_center, new_y_top,
		this->width, this->height,
		this->mincount, this->lane_found);
}

bool WindowBox::has_lane(void)
{
	if (!lane_found && has_line()) lane_wblfound = true;
	return lane_found;
}

std::ostream& operator<<(std::ostream& out, WindowBox const& window)
{
	out << "Window Box [" << window.x_left << ", " << window.y_bottom << ", ";
	out << window.x_right << ", " << window.y_top << "]" << std::endl;

	return out;
}*/


void window_search(cv::Mat &line_binary, cv::Mat &histogram, std::vector<WindowBox>& left_boxes, std::vector<WindowBox>& right_boxes, int n_windows, int window_width) {

	cv::Point peak_left, peak_right;
	lane_peaks(histogram, peak_left, peak_right); // Peaks

	int window_height = line_binary.rows / n_windows;

	peak_left.y = line_binary.rows - window_height/2;
	peak_right.y = line_binary.rows - window_height/2;

	//std::cout << "starting at: l" << peak_left << "  r" << peak_right << std::endl;

	// Initialise left and right window boxes
	WindowBox wbl(peak_left,  window_width, window_height);
	WindowBox wbr(peak_right, window_width, window_height);

	find_lane_windows(line_binary, wbl, left_boxes);
	find_lane_windows(line_binary, wbr, right_boxes);

	// Parallelize searching
	//std::thread left(find_lane_windows, std::ref(line_binary), std::ref(wbl), std::ref(left_boxes));
	//std::thread right(find_lane_windows, std::ref(line_binary), std::ref(wbr), std::ref(right_boxes));
	//left.join();
	//right.join();

}

void find_lane_windows(cv::Mat& binary_img, WindowBox &window_box, std::vector<WindowBox>& wboxes)
{
	window_box.find_lane(binary_img);
	wboxes.push_back(window_box);
	cv::Point next_p_start = window_box.get_next_start();
	while(next_p_start.y > 0) {
		WindowBox w_new(next_p_start, window_box.get_width(), window_box.get_height());
		w_new.find_lane(binary_img);
		wboxes.push_back(w_new);
		next_p_start = w_new.get_next_start();
		//std::cout << "next start: " << next_p_start << std::endl;
	}

	/*bool continue_lane_search = true;
	int contiguous_box_no_line_count = 0;

	// keep searching up the image for a lane lineand append the boxes
	while (continue_lane_search && window_box.y_top > 0) {
		if (window_box.has_line())
			wboxes.push_back(window_box);
		window_box = window_box.get_next_windowbox(binary_img);

		// if we've found the lane and can no longer find a box with a line in it
		// then its no longer worth while searching
		if (window_box.has_lane()) {
			if (window_box.has_line()) {
				contiguous_box_no_line_count = 0;
			} else {
				contiguous_box_no_line_count += 1;
				if (contiguous_box_no_line_count >= 4)
					continue_lane_search = false;
			}
		}
	}*/

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
			cv::rectangle(img, box.get_window_rect(img), cv::Scalar(0, 255, 0), 2);
			cv::circle(img, box.get_center(), 2, cv::Scalar(0,0,255), 2);
		}
	}
}


void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order) {
	CV_Assert((src_x.rows > 0) && (src_y.rows > 0) && (src_x.cols == 1) && (src_y.cols == 1)
		&& (dst.cols == 1) && (dst.rows == (order + 1)) && (order >= 1));
	cv::Mat X;
	X = cv::Mat::zeros(src_x.rows, order + 1, CV_32FC1);
	cv::Mat copy;
	for (int i = 0; i <= order; i++)
	{
		copy = src_x.clone();
		pow(copy, i, copy);
		cv::Mat M1 = X.col(i);
		copy.col(0).copyTo(M1);
	}
	cv::Mat X_t, X_inv;
	transpose(X, X_t);
	cv::Mat temp = X_t * X;
	cv::Mat temp2;
	invert(temp, temp2);
	cv::Mat temp3 = temp2 * X_t;
	cv::Mat W = temp3 * src_y;
	W.copyTo(dst);
}
