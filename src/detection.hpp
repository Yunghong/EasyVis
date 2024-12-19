#ifndef DETECTION_H
#define DETECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "yolov8.h"
#include "camera.hpp"
#include "container.hpp"

// ByteTrack
#include "BYTETracker.h"

class Detection{
private:
    YoloV8 yoloV8;
    
    // A paper about this tracker work in preparation 
    // std::vector<BYTETracker> tracker;
    // std::vector<BYTETrackerBean> trackerBean;
    // std::vector<BYTETrackerGrasper> trackerGrasper;
    
public:
    std::vector<std::vector<Object>> objects;

    std::vector<std::vector<bean2d>> Bean_Black;
    std::vector<std::vector<bean2d>> Bean_DarkBlue;
    std::vector<std::vector<bean2d>> Bean_Gray;
    std::vector<std::vector<bean2d>> Bean_Green;
    std::vector<std::vector<bean2d>> Bean_LightBlue;
    std::vector<std::vector<bean2d>> Bean_Orng;
    std::vector<std::vector<bean2d>> Bean_Prpl;
    std::vector<std::vector<bean2d>> Bean_Red;

    std::vector<std::vector<grasper2d>> Grasper_B;
    std::vector<std::vector<grasper2d>> Grasper_W;
    
    // A paper about this tracker work in preparation 
    // std::vector<std::vector<STrackBean>> beanTrackings;
    // std::vector<std::vector<STrackGrasper>> grasperTrackings;
    
    Detection(const std::string& onnxModelPath, const YoloV8Config& config, Cameras *cameras);

    ~Detection();

    void detect(std::vector<cv::Mat> imgs);
};

#endif
