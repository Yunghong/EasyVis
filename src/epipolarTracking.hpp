// A paper about this work is still in preparation, temporary remove detail

#ifndef EPIPOLARTRACKING_H
#define EPIPOLARTRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_map>
#include <unordered_set>

#include "yolov8.h"
#include "camera.hpp"
#include "container.hpp"


// ByteTrack
#include "BYTETracker.h"

class EpipolarTrackor{
public:
    int viewNumber;
    int maxIndexGrasper;
    int maxIndexBean;
    int encodeVal;

    std::vector<BYTETracker> tracker;
    std::vector<BYTETrackerBean> trackerBean;
    std::vector<BYTETrackerGrasper> trackerGrasper;
    
    std::vector<std::vector<STrackBean>> beanTrackings;
    std::vector<std::vector<STrackGrasper>> grasperTrackings;
    
    EpipolarTrackor(Cameras *cameras);

    ~EpipolarTrackor();

    void track(std::vector<std::vector<Object>> objects);

    double getDistance(double x1, double y1, double a, double b, double c);
    cv::Mat getF(const cv::Mat& Ra, const cv::Mat& Rb, const cv::Mat& ta, const cv::Mat& tb, const cv::Mat& K);
    std::pair<cv::Point2i, cv::Point2i> getLine(const cv::Mat& l);

    void epipolarMatchingBean(  const std::vector<std::vector<bean2d>> &beansMV, 
                        const std::vector<cv::Mat>& Rs, const std::vector<cv::Mat>& ts, const cv::Mat& K, 
                        std::vector<std::vector<int>>& groups);

    void epipolarMatchingBeanDetectResult(  const std::vector<std::vector<bean2d>> &beansMV, 
                        const std::vector<cv::Mat>& Rs, const std::vector<cv::Mat>& ts, const cv::Mat& K, 
                        std::vector<std::vector<int>>& groups);

    void epipolarMatchingGrasper(  const std::vector<std::vector<grasper2d>> &graspersMV, 
                        const std::vector<cv::Mat>& Rs, const std::vector<cv::Mat>& ts, const cv::Mat& K, 
                        std::vector<std::vector<int>>& groups);

    double calculateDistance(double x1, double y1, double x2, double y2);
    void associateBean(std::vector<std::vector<int>>& groups, std::unordered_set<int> &beanIds);
    void associateGrasper(std::vector<std::vector<int>> groups, std::unordered_set<int> &grasperIds);
};


#endif
