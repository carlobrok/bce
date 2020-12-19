#ifndef LINE_CALCULATION
#define LINE_CALCULATION

#include "lane_data.hpp"

#include <opencv2/opencv.hpp>
#include <cmath>

void window_search(cv::Mat &line_binary, cv::Mat & histogram, std::vector<WindowBox>& boxes_left, std::vector<WindowBox>& boxes_right, int n_windows, cv::Size window_size);

void lane_peaks(cv::Mat const& histogram, cv::Point& left_max_loc, cv::Point& right_max_loc);

void find_lane_windows(cv::Mat& binary_img, WindowBox &window_box, std::vector<WindowBox>& wboxes, int n_windows);

void draw_boxes(cv::Mat& img, const std::vector<WindowBox>& boxes);

/**
 * @brief Berechnet die linken und rechten Mittelpunkte mit der ROI-Methode. Dazu wird das Binärbild entlang der Mittellinie geteilt und jeweils links und rechts in horizontale ROIs aufgeteilt. In den einzelnen ROIs wird dann der Mittelpunkt berechnet. Mit den Mittelpunkten werden die lane Daten neu gesetzt.
 * 
 * @param binary_line Binärbild der Linie
 * @param lane_mid Berechnete Mittellinie
 * @param lane_left setzt linke Linie neu
 * @param lane_right setzt rechte Linie neu
 * @param roi_height höhe der ROIs
 * @param n_rois Anzahl an ROIs übereinander
 * @param min_area_size Schwellwert, unter dem der Center der Fläche im ROI nicht berechnet wird (Vermeidung Bildsrauschen)
 */
void roi_search(cv::Mat & binary_line, cv::Mat & vis_draw, lane_data & lane_mid, lane_data & lane_left, lane_data & lane_right, int roi_height, int n_rois, int min_area_size = 30);

lane_data calc_midline(lane_data & left, lane_data & right, cv::Size image_size);

// obsolete
void calc_midpoints(const std::vector<WindowBox>& boxes_left, const std::vector<WindowBox>& boxes_right, std::vector<cv::Point> & midpoints);


#endif
