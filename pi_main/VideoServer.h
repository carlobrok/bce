/*
 * VideoServer.h
 *
 *  Created on: Sep 2, 2017
 *      Author: joern
 */

#ifndef VIDEOSERVER_H
#define VIDEOSERVER_H

#include "opencv2/opencv.hpp"
#include <string>

namespace srv {

	void init(bool l_asyncSend = false);

	void namedWindow(const std::string & window);
	void imshow(const std::string & window, const cv::Mat & image);
	void update();

	void run();
	void waitForConnection();

}

#endif /* VIDEOSERVER_H_ */
