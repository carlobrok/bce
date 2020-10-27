#ifndef CALC_POLY
#define CALC_POLY

#include "opencv2/opencv.hpp"
#include <vector>

inline std::vector<cv::Point> warp_poly_points;

struct sort_y{
    bool operator() ( cv::Point2f a, cv::Point2f b ){
        if ( a.y != b.y ) 
            return a.y < b.y;
        return a.x <= b.x ;
    }
};

struct sort_x{
    bool operator() ( cv::Point2f a, cv::Point2f b ){
        if ( a.x != b.x ) 
            return a.x < b.x;
        return a.y <= b.y ;
    }
};

void calc_transform(cv::Mat & input);

#endif
