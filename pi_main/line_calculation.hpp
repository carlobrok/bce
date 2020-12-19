#ifndef LINE_CALCULATION
#define LINE_CALCULATION

#include <opencv2/opencv.hpp>
#include <cmath>
#include <ostream>

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
void roi_search(cv::Mat & binary_line, lane_data & lane_mid, lane_data & lane_left, lane_data & lane_right, int roi_height, int n_rois, int min_area_size = 30);




lane_data calc_midline(lane_data left, lane_data right, cv::Size image_size);

class lane_data {
public: 

	cv::Vec2f vec;
	cv::Point2f point;
	char id = 'n';

	int image_height = 480;

	lane_data() { vec.zeros(); };
	lane_data(cv::Vec4f line, char id = 'n', int image_height = 480) : vec( line[0],line[1] ), point( line[2],line[3] ), image_height(image_height), id(id) {};
	lane_data(cv::Vec2f vec, cv::Point point, char id = 'n', int image_height = 480) : vec(vec) , point(point), image_height(image_height), id(id) {};

	void set_data(cv::Vec2f vec, cv::Point point) { this->vec = vec; this->point = point; };

	/**
	 * @brief Setzt Punkt und Richtungsvektor der Klasse anhand der Ausgleichsgerade der centerpoints neu
	 * 
	 * @param center_points		vector of points defining a line
	 * @param min_points  		Anzahl, an Punkten, die mindestens vorhanden sein muss, um die Linie sicher bestimmen zu können
	 * @param reset 			setzt Punkt und Richtungsvektor zurück
	 */
	void set_data(std::vector<cv::Point>& boxes, int min_points = 3, bool reset = true);

	/** 
	 * @brief Überladene Funktion, welche zuherst die Mittelpunkte der WindowBoxes in einen vector schreibt, welcher an center_points übergeben wird
	 * 
	 * @param boxes  			vector of WindowBoxes, output of window_search
	 * @param min_points 		Anzahl, an Punkten, die mindestens vorhanden sein muss, um die Linie sicher bestimmen zu können
	 * @param reset 	  		setzt Punkt und Richtungsvektor zurück
	 */
	void set_data(std::vector<WindowBox>& boxes, int min_points = 3, bool reset = true);

	/**
	 * @brief Leert die Daten der Klasse, has_lane ist dann false.
	 * 
	 */
	void clear() { vec.zeros(); };

	/**
	 * @brief Calculates the point on the line at the top (y=0) of the image.
	 * 
	 * @return cv::Point 
	 */
	cv::Point top();

	/**
	 * @brief Calculates the point on the line for a given y value
	 * 
	 * @param y gegebener Y-Wert, für den der Punkt bestimmt wird
	 * @return cv::Point 
	 */
	cv::Point at_y(int y);
	
	/**
	 * @brief Calculates the point on the line at the bottom (y=image_height) of the image. 
	 * 
	 * @return cv::Point 
	 */
	cv::Point bottom();
	
	/**
	 * @brief Berechnet den Winkel der Linie zur Vertikalen. 
	 * 
	 * @return float 
	 */
	float angle() { return atan( - vec[0] / vec[1] ) * 180 / 3.142; }

	/**
	 * @brief Gibt true zurück, wenn Daten vorhanden sind, die eine Lane repräsentieren
	 * 
	 * @return bool
	 */
	bool has_lane() { return ( cv::countNonZero(vec) > 0 ); }
	
	double distance(lane_data & line2, double angle_factor = 5);

	void draw( cv::Mat & img, cv::Scalar color = cv::Scalar(0,0,255));
	friend std::ostream& operator<<(std::ostream& os, const lane_data& ln);
};


// obsolete
void calc_midpoints(const std::vector<WindowBox>& boxes_left, const std::vector<WindowBox>& boxes_right, std::vector<cv::Point> & midpoints);


#endif
