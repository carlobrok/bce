#ifndef LANE_DATA
#define LANE_DATA

#include "window_box.hpp"

#include <opencv2/opencv.hpp>
#include <ostream>


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


#endif