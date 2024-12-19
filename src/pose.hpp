// load camera pose. The pose describes 3D info of cameras in the 3D space

#ifndef POSE_H
#define POSE_H

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "config.hpp"

class CameraPose
{
private:

public:
    std::vector<cv::Mat> projMatrices;

    //K, R, C, T
    //std::vector<std::vector<cv::Mat>> cameraMatrices;
    std::vector<cv::Mat> Ks, Rs, Cs, Ts;
    
    CameraPose(EasyVisConfig* config);

    void DecomposeProjectionMatrices();  
    void SaveKRts(std::string dataRoot);
    std::vector<std::vector<double>> readProjectionMatrix(std::string fileName);

    ~CameraPose();
};

#endif
