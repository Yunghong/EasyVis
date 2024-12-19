// A paper about this work is still in preparation, temporary remove detail

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_map>
#include <unordered_set>

#include "yolov8.h"
#include "camera.hpp"
#include "container.hpp"
#include "epipolarTracking.hpp"
#include "util.hpp"

// ByteTrack
#include "BYTETracker.h"

EpipolarTrackor::EpipolarTrackor(Cameras *cameras): 
        tracker(cameras->viewNumber, BYTETracker(30, 30)),
        trackerBean(cameras->viewNumber, BYTETrackerBean(30, 30)),
        trackerGrasper(cameras->viewNumber, BYTETrackerGrasper(30, 30)){

}

EpipolarTrackor::~EpipolarTrackor(){}

void EpipolarTrackor::track(std::vector<std::vector<Object>> objects){
   
}

// Function to compute the distance from a point to a line
double EpipolarTrackor::getDistance(double x1, double y1, double a, double b, double c) {
}

// Function to get the fundamental matrix
cv::Mat EpipolarTrackor::getF(const cv::Mat& Ra, const cv::Mat& Rb, const cv::Mat& ta, const cv::Mat& tb, const cv::Mat& K) {
}

// Function to get the line equation from the fundamental matrix
std::pair<cv::Point2i, cv::Point2i> EpipolarTrackor::getLine(const cv::Mat& l) {
}

void EpipolarTrackor::epipolarMatchingBeanDetectResult(  const std::vector<std::vector<bean2d>> &beansMV, 
                        const std::vector<cv::Mat>& Rs, const std::vector<cv::Mat>& ts, const cv::Mat& K, 
                        std::vector<std::vector<int>>& groups) { 
}

void EpipolarTrackor::epipolarMatchingBean(  const std::vector<std::vector<bean2d>> &beansMV, 
                        const std::vector<cv::Mat>& Rs, const std::vector<cv::Mat>& ts, const cv::Mat& K, 
                        std::vector<std::vector<int>>& groups) {
}


void EpipolarTrackor::epipolarMatchingGrasper(  const std::vector<std::vector<grasper2d>> &graspersMV, 
                        const std::vector<cv::Mat>& Rs, const std::vector<cv::Mat>& ts, const cv::Mat& K, 
                        std::vector<std::vector<int>>& groups) {
 
}

double EpipolarTrackor::calculateDistance(double x1, double y1, double x2, double y2) {
}

void EpipolarTrackor::associateGrasper(std::vector<std::vector<int>> groups, std::unordered_set<int> &grasperIds){
}


void EpipolarTrackor::associateBean(std::vector<std::vector<int>>& groups, std::unordered_set<int> &beanIds){
}
