#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "config.hpp"

class Camera
{
private:
	std::string address;
	int port;
	cv::VideoCapture capturer;
	
public:
	// Constructor
	Camera(const std::string& address, EasyVisConfig* config);
	Camera(int port, EasyVisConfig* config);
	
	// Read image
	void capture(cv::Mat &img);
	
	// Destructor
	~Camera();
};

class Cameras
{
private:
	std::vector<Camera> cams;

public:
	std::vector<cv::Mat> imgs;
	int viewNumber;

	// Constructor initializes camera arrays
	// record mod
	Cameras(const std::vector<std::string>& addresses, EasyVisConfig* config);
	
	// online mod
	Cameras(const std::vector<int>& port, EasyVisConfig* config);
	
	// read images
	bool captures();
	
	// Destructor
	~Cameras();
};
#endif
