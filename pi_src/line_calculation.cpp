#include "line_calculation.hpp"

#include <opencv2/opencv.hpp>
#include <thread>


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

		double sum = 0;
		for (auto const& point : nonzero) {
			sum += (point.x + x_left);
		}
		new_x_center = sum / nonzero.size(); // recenter based on mean
	}
	if (new_x_center + this->width / 2 > binary_img.cols) return WindowBox(); // if outside of ROI return empty box

	return WindowBox(binary_img, new_x_center, new_y_top,
		this->width, this->height,
		this->mincount, this->lane_found);
}

bool WindowBox::has_lane(void)
{
	if (!lane_found && has_line()) lane_found = true;
	return lane_found;
}

std::ostream& operator<<(std::ostream& out, WindowBox const& window)
{
	out << "Window Box [" << window.x_left << ", " << window.y_bottom << ", ";
	out << window.x_right << ", " << window.y_top << "]" << std::endl;

	return out;
}


void window_search(cv::Mat &warped, cv::Mat &histogram, std::vector<WindowBox>& left_boxes, std::vector<WindowBox>& right_boxes, int n_windows, int window_width) {

	cv::Point peak_left, peak_right;
	lane_peaks(histogram, peak_left, peak_right); // Peaks

	// Initialise left and right window boxes
	WindowBox wbl(warped, peak_left.x, warped.rows, window_width, warped.rows / n_windows);
	WindowBox wbr(warped, peak_right.x, warped.rows, window_width, warped.rows / n_windows);

	// Parallelize searching
	std::thread left(find_lane_windows, std::ref(warped), std::ref(wbl), std::ref(left_boxes));
	std::thread right(find_lane_windows, std::ref(warped), std::ref(wbr), std::ref(right_boxes));

	left.join();
	right.join();

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


void find_lane_windows(cv::Mat& binary_img, WindowBox& window_box, std::vector<WindowBox>& wboxes)
{
	bool continue_lane_search = true;
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
	}

	return;
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



void draw_boxes(cv::Mat& img, const std::vector<WindowBox>& boxes) {
	// Draw the windows on the output image
	for (const auto& box : boxes) {
		cv::rectangle(img, box.get_bottom_left_point(), box.get_top_right_point(), cv::Scalar(0, 255, 0), 2);
	}
}
