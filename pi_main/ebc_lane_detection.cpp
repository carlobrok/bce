#include "devices.hpp"
#include "img_processing.hpp"
#include "line_calculation.hpp"
#include "drive_calculation.hpp"
#include "VideoServer.h"
#include "CameraCapture.h"
#include "lane_data.hpp"
#include "window_box.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>

#include <chrono>
#include <thread>

#include <csignal>

const int n_windows = 12;



void signalHandler( int signum ) {
    std::cout << "Signal (" << signum << ") received: stopping car" << std::endl;

    mot::set_state(mot::OFF, true);
    // cleanup and close up stuff here  
    // terminate program  

    exit(signum);  
}

int main() {

    srv::init(true);				// init VideoServer

    CameraCapture cam(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
	//cam.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	//cam.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
	cam.set(cv::CAP_PROP_FPS, 30);			// Kamera Framerate auf 30 fps


	srv::namedWindow("input image");
	//srv::namedWindow("warped");
	srv::namedWindow("binary_line");
	// only for color filtering
	srv::namedWindow("hsv");
	srv::namedWindow("color_thresh");
	// only for sobel filtering
	/*srv::namedWindow("hls");
	srv::namedWindow("sobel_grad");
	srv::namedWindow("sobel_thresh");*/

	init_arduino(0x08);

	// hande interrupt and segmentation fault to brake motor
	signal(SIGINT, signalHandler);
	signal(SIGSEGV, signalHandler);

	cv::Mat bgr, binary_line, histogram;

	lane_data lane_left, lane_right, lane_mid;
	lane_data lane_left_old, lane_right_old;


    auto last_midlane_found = std::chrono::system_clock::now();

    int speed = 0;
    double angle = 0;
	
    while(1) {

        auto tstart = std::chrono::system_clock::now();

    // ======== image processing pipeline ========

    // load image

        while(!cam.read(bgr)){}
        //cv::resize(bgr, bgr, cv::Size(1000, 600));
        cv::GaussianBlur(bgr, bgr, cv::Size(5,5),2,2);		// Gaussian blur to normalize image

        auto timg_read = std::chrono::system_clock::now();
        std::cout << "image read: " << std::chrono::duration_cast<std::chrono::milliseconds>(timg_read - tstart).count() << "ms" << std::endl;
        //TODO: distortion correction

    // generate binary:

        /*// perspective_warp
        perspective_warp(bgr, warped, transform_M);
        auto tpersp_warp = std::chrono::system_clock::now();
        std::cout << "perspective_warp: " << std::chrono::duration_cast<std::chrono::milliseconds>(tpersp_warp - timg_read).count() << "ms" << std::endl;
        */

        // color filtering
        color_filtering(bgr, binary_line, cv::Scalar(115,53,105), cv::Scalar(205, 220, 205));
        srv::imshow("binary_line", binary_line);
        auto tcolor = std::chrono::system_clock::now();
        std::cout << "color filtering: " << std::chrono::duration_cast<std::chrono::milliseconds>(tcolor - timg_read).count() << "ms" << std::endl;

        /*
        // sobel filtering
        sobel_filtering(warped, binary_line, 20, 255);
        srv::imshow("binary_line", binary_line);
        auto tsobel = std::chrono::system_clock::now();
        std::cout << "sobel: " << std::chrono::duration_cast<std::chrono::milliseconds>(tsobel - tpersp_warp).count() << "ms" << std::endl;
        */

        // (IDEA more binary filtering)

        // wenn schon eine midlane existiert:
        // Bild entlang der Midlane trennen und in horizontale ROIs links und rechts einteilen. In diesen ROIs je nach einer neuen Mittellinie suchen.
        
        if ( lane_mid.has_lane() ) {        // wenn mindestens eine 

            std::cout << "roi search" << std::endl;

            // alte lane_datas zwischenspeichern
            lane_left_old = lane_left;
            lane_right_old = lane_right;

            // ROI search Methode
            
            roi_search(binary_line, lane_mid, lane_left, lane_right, binary_line.rows / 20, n_windows);
                    
            /*IDEA: 
            sprünge verhindern:
            vorhersehen, wo die linie sein müsste. 
            Sonst würde eine falsche Linie spätestens im 2. Bild auftauchen. 
            Aber: trennen durch midlane verhindert Springen der lane auf gegenüberliegende Seite
            */
            // je maximale Abweichung zu letzer lane ermitteln:  abweichung zu groß?
                // prediction erstellen
                // variable prediction auf true setzen
                
            
            // calc midlane
            // lanes links und rechts gefunden? : calculate midlane
            if (lane_left.has_lane() && lane_right.has_lane()) {
                lane_mid = calc_midline(lane_left, lane_right, binary_line.size());
            }


            // VERBESSERUNG: Mittellinie um den Winkel drehen, wie sich auch links/rechts gedreht hat

            // sonst: nur links gefunden? : midline mit gleichem abstand setzen, wie zuvor zu links
            else if (lane_left.has_lane()) {
                lane_mid.set_data(lane_mid.vec, lane_left.point + lane_mid.point - lane_left_old.point);
            }
            // sonst: nur rechts gefunden? :  midline mit gleichem abstand setzen, wie zuvor zu rechts
            else if (lane_right.has_lane()) {
                lane_mid.set_data(lane_mid.vec, lane_right.point + lane_mid.point - lane_right_old.point);
            } 

            // timeout:
            // links oder rechts eine lane gefunden?
            if (lane_left.has_lane() || lane_right.has_lane()) {
                // last midlane zeit setzen
                last_midlane_found = std::chrono::system_clock::now();
            // sonst wenn last midlane zeit zu lange her ist (z.B. > 1s) 
            } else if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_midlane_found).count() > 1000) {
                // midlane data leeren (-> windowsearch)
                lane_mid.clear();
            }
            
                
            lane_right.draw(bgr, cv::Scalar(255,0,255));
            lane_left.draw(bgr, cv::Scalar(255,0,255));

        }

        // Wenn noch keine midlane midlane gefunden wurde:
        // eine neue midlane nach der window_search-Methode suchen
        else {
        
            std::cout << "window search" << std::endl;

            // histogram peak detection
            lane_histogram(binary_line, histogram, binary_line.rows/2);
            
            std::vector<WindowBox> boxes_left, boxes_right;
            
            window_search(binary_line, histogram, boxes_left, boxes_right, n_windows, cv::Size(200, binary_line.rows / 20));

            std::cout << "lbs " << boxes_left.size() << " rbs " << boxes_right.size() << std::endl;

            lane_left.set_data(boxes_left);
            lane_right.set_data(boxes_right);

            draw_boxes(bgr, boxes_left);
            draw_boxes(bgr, boxes_right);

            // nur midlane berechnen, wenn links und rechts eine Lane gefunden wurde
            if( lane_left.has_lane() && lane_right.has_lane() ) { 
                lane_mid = calc_midline(lane_left, lane_right, bgr.size());
 
                last_midlane_found = std::chrono::system_clock::now();                             // last midlane zeit setzen
            }

            lane_right.draw(bgr);
            lane_left.draw(bgr);
        }
        
        std::cout << "line_left: " << lane_left << " line_right" << lane_right << std::endl;
    
        lane_mid.draw(bgr);

        std::cout << "lane mid vec: "  << lane_mid << std::endl; 

        auto tline_calc = std::chrono::system_clock::now();
        std::cout << "calculate lines: " << std::chrono::duration_cast<std::chrono::milliseconds>(tline_calc - tcolor).count() << "ms" << std::endl;

    // ========= autonomous driving ========

        
        if (lane_mid.has_lane()) {

            // calculate speed from midpoints
            // int speed = calc_speed(midpoints);
            speed = 50;

            double diff_mid = (lane_mid.point.x - binary_line.cols/2);
            std::cout << "diff to mid: " <<  diff_mid << std::endl;
            
            // calculate steering
            angle = lane_mid.angle() + (diff_mid / 6);

            std::cout << "speed, angle: " << speed << ", " << angle << std::endl;
        } else {   
            speed = 0;
            std::cout << "no midlane, speed=0, angle=" << angle << " (remains)" << std::endl;
        }

        // TODO request for obstacles/state - hande them

        // if no ground pause until ground available:
            // check n more times for ground then
            // continue;

        // TODO if no obstacles send speed and steering to arduino

        int e = mot::set_dir_pwm_steer(mot::FORWARD, speed, (int)angle);
        if(e < 0) {
            std::cout << "Error sending i2c data: " << strerror(errno) << std::endl;
        } else {
            std::cout << "Successfully sent i2c data" << std::endl;
        }

    // output images:

        // TODO draw overlay

        //srv::imshow("warped", warped);
        srv::imshow("input image", bgr);

        // send/display video
        //srv::imshow("histogram", histogram);

        auto tend = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tend - tstart).count();
        std::cout << "whole proc: " << ms << "ms  / " << 1000 / ms << "fps" <<  std::endl;

        std::cout << "-------------------------------" << std::endl;
    }

  return 0;
}
