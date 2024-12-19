#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camera.hpp"
#include "config.hpp"

// this module handle multi-view camera capture

Camera::Camera(const std::string& address, EasyVisConfig* config) : address(address) {
    capturer.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // If don't work, change V4L2 to ANY
    capturer.open(address, cv::CAP_V4L2);
    // capturer.open(address);
}

Camera::Camera(int port, EasyVisConfig* config) : port(port) {
    capturer.open(port, cv::CAP_V4L2);
    capturer.set(cv::CAP_PROP_BUFFERSIZE, 1);
    capturer.set(cv::CAP_PROP_FRAME_HEIGHT, config->CAM_RES_HEIGHT);
    capturer.set(cv::CAP_PROP_FRAME_WIDTH, config->CAM_RES_WIDTH);
    capturer.set(cv::CAP_PROP_FPS, config->CAM_FPS);
    capturer.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
}

void Camera::capture(cv::Mat& img) {
    capturer.read(img);
}

Camera::~Camera() {}

// init cams, record mod
Cameras::Cameras(const std::vector<std::string>& addresses, EasyVisConfig* config) {
    viewNumber = config->VIEW_NUMBER;
    // #pragma omp parallel for
    for (const auto& address : addresses) {
        cams.emplace_back(address, config);
    }

    imgs.resize(viewNumber);
}

// init cams, online mod
Cameras::Cameras(const std::vector<int>& ports, EasyVisConfig* config) {
    viewNumber = ports.size();

    for (const auto& port : ports) {
        cams.emplace_back(port, config);
    }

    imgs.resize(viewNumber);
}

// cam read
bool Cameras::captures() {
    bool valid=true;

    #pragma omp parallel for
    for (int i = 0; i < viewNumber; i++) {
        cams[i].capture(imgs[i]);
        valid=valid & !imgs[i].empty();
    }

    return valid;
}

Cameras::~Cameras() {}
