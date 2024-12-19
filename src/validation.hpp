// this module calculates the testing scores using BPE.

#ifndef VALIDATION_H
#define VALIDATION_H

#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "renderer.hpp"
#include "shader.hpp"
#include "glmodel.hpp"
#include "texture.hpp"
#include "pose.hpp"
#include "glmodel.hpp"
#include "container.hpp"
#include "kalmanfilter.hpp"

class Validation{
    CameraPose *cameraPose;
    int viewNumber;
    
    std::vector<float> grasperJoint;
    std::vector<float> grasperLocator;
    std::vector<float> grasperFinger;
    std::vector<float> beanCenter;
    std::vector<float> average;
    float grasperJointCounter;
    float grasperLocatorCounter;
    float grasperFingerCounter;
    float beanCenterCounter;
    float averageCounter;

public:
    Validation(CameraPose *cameraPose, int views);
    
    float getDistance(cv::Point2i groundTruth, cv::Point2f backProjected);
    float sumVector(std::vector<float> input);

    cv::Point2f backProject(const cv::Matx34d &P, std::vector<float>  &W);
    void BPEAppend(
            std::unordered_map<int, grasper3dKF> &graspersKF3d, 
            std::unordered_map<int, bean3dKF> &beansKF3d,
            std::vector<std::vector<grasper2d>> grasper,
            std::vector<std::vector<bean2d>> bean);

    void BPESummary();
};

#endif
