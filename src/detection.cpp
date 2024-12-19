#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

// ByteTrack
#include "BYTETracker.h"

#include "yolov8.h"
#include "detection.hpp"
#include "camera.hpp"
#include "container.hpp"

// init detector
Detection::Detection(const std::string& onnxModelPath, const YoloV8Config& config, Cameras *cameras): 
        yoloV8(onnxModelPath, config){
    cameras->captures();

    // try this
    objects.resize(cameras->imgs.size());

    Bean_Black.resize(cameras->imgs.size());
    Bean_DarkBlue.resize(cameras->imgs.size());
    Bean_Gray.resize(cameras->imgs.size());
    Bean_Green.resize(cameras->imgs.size());
    Bean_LightBlue.resize(cameras->imgs.size());
    Bean_Orng.resize(cameras->imgs.size());
    Bean_Prpl.resize(cameras->imgs.size());
    Bean_Red.resize(cameras->imgs.size());

    Grasper_B.resize(cameras->imgs.size());
    Grasper_W.resize(cameras->imgs.size());
    
}

Detection::~Detection(){}

// 2D pose estimation
void Detection::detect(std::vector<cv::Mat> imgs){

    for(int i=0;i<imgs.size();i++){
        objects[i]=yoloV8.detectObjects(imgs[i]);

        yoloV8.summaryResults(
            objects[i],
            Bean_Black[i],
            Bean_DarkBlue[i],
            Bean_Gray[i],
            Bean_Green[i],
            Bean_LightBlue[i],
            Bean_Orng[i],
            Bean_Prpl[i],
            Bean_Red[i],
            Grasper_B[i],
            Grasper_W[i]);
    }    
}
