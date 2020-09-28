#include "../pi_main/VideoServer.h"

#include <iostream>
#include "opencv2/opencv.hpp"
#include "boost/asio.hpp"
#include <thread>
#include <chrono>

int main(int argc, char *argv[]) {

  if(argc != 2){
		std::cout << "Es muss die IP-Adresse des Video Servers angegeben werden!" << std::endl;
		std::cout << "Aufruf: ./VideoClient IP_ADRESSE" << std::endl;
		return -1;
	}


	//Erstelle cv::Mat für Eingabebild
	cv::Mat img_rgb;

	/////////
	//Boost TCP Client
	////////
	boost::asio::io_service ios;

	//boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string("192.168.2.121"), 1300);
	//boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string("192.168.2.119"), 1300);
	//boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 1300);
	boost::asio::ip::tcp::endpoint endpoint;

	try{
		endpoint = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(argv[1]), 1300);
	}
	catch(std::exception & e){
		std::cerr << e.what() << std::endl;
		std::cout << "IP-Adresse nicht korrekt übergeben?" << std::endl;
		return -1;
	}

	boost::asio::ip::tcp::socket socket(ios);

	bool connected = false;
	while(!connected){
		try{
			socket.connect(endpoint);
			connected = true;
		}
		catch(std::exception & e){
			std::cout << "No server" << std::endl;
			std::this_thread::sleep_for (std::chrono::seconds(5));
		}


	}


	while(1){
		//int64_t t1 = cv::getTickCount();
		try{
			uint32_t sz;
			boost::asio::read(socket, boost::asio::buffer(&sz, sizeof(sz)));


			std::vector<char> nameBuf;
			nameBuf.resize(20);
			boost::asio::read(socket, boost::asio::buffer(nameBuf, 20));

			std::string name(nameBuf.data(), std::strlen(nameBuf.data()));

			std::cout << "Window: " << name << std::endl;

			std::vector<uchar> img_buf;
			img_buf.resize(sz);
			boost::asio::read(socket, boost::asio::buffer(img_buf.data(), sz));
			int64 t1 = cv::getTickCount();
			cv::Mat img_rgb = cv::imdecode(img_buf, 1);
			int64 t2 = cv::getTickCount();

			std::cout << "Decode took " << (t2-t1) / cv::getTickFrequency() * 1000.0 << std::endl;

			//Zeige eingelesenes Bild "img_rgb" im Fenster "RGB" an
			cv::imshow(name, img_rgb);

			cv::waitKey(1);
		}
		catch(std::exception & e){
			std::cerr << e.what() << std::endl;
			std::cout << "Connection broken. Trying to reconnect" << std::endl;
			socket.close();
			cv::destroyAllWindows();
			bool connected = false;
			while(!connected){
				try{
					socket.connect(endpoint);
					connected = true;
				}
				catch(std::exception & e){
					std::cout << "No server" << std::endl;
					std::this_thread::sleep_for (std::chrono::seconds(1));
				}


			}
		}
		//int64_t t2 = cv::getTickCount();
		//std::cout << "FPS: " << 1/((t2-t1)/cv::getTickFrequency()) << std::endl;
	}


  return 0;
}