// This code was modified and extended based on: https://github.com/cyrusbehr/YOLOv8-TensorRT-CPP
// The modifications are made to fit EasyVis objects and framework 

#pragma once
#include "engine.h"
#include <fstream>
#include "container.hpp"

// Utility method for checking if a file exists on disk
inline bool doesFileExist (const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

struct Object {
    // The object class.
    int label{};
    // The detection's confidence probability.
    float probability{};
    // The object bounding box rectangle.
    cv::Rect_<float> rect;
    // Semantic segmentation mask
    cv::Mat boxMask;
    // Pose estimation key points
    std::vector<float> kps{};
};

// Config the behavior of the YoloV8 detector.
// Can pass these arguments as command line parameters.
struct YoloV8Config {
    // The precision to be used for inference
    Precision precision = Precision::FP16;
    // Calibration data directory. Must be specified when using INT8 precision.
    std::string calibrationDataDirectory;
    // Probability threshold used to filter detected objects
    float probabilityThreshold = 0.9f;
    // Non-maximum suppression threshold
    float nmsThreshold = 0.8f;
    // Max number of detected objects to return
    int topK = 100;
    // Segmentation config options
    int segChannels = 32;
    int segH = 160;
    int segW = 160;
    float segmentationThreshold = 0.5f;
    // Pose estimation options
    int numKPS = 4;
    float kpsThreshold = 0.8f;
    // Class thresholds (default are COCO classes)
    std::vector<std::string> classNames = {
        "Bean_Black","Bean_DarkBlue","Bean_Gray","Bean_Green","Bean_LightBlue","Bean_Orng","Bean_Prpl","Bean_Red","Grasper_B","Grasper_W"
    };
};

class YoloV8 {
public:
    // Builds the onnx model into a TensorRT engine, and loads the engine into memory
    YoloV8(const std::string& onnxModelPath, const YoloV8Config& config);

    // Detect the objects in the image
    std::vector<Object> detectObjects(const cv::Mat& inputImageBGR);
    std::vector<Object> detectObjects(const cv::cuda::GpuMat& inputImageBGR);

    // Draw the object bounding boxes and labels on the image
    void drawObjectLabels(cv::Mat& image, const std::vector<Object> &objects, unsigned int scale = 2);
    void summaryResults(
        const std::vector<Object> &objects, 
        std::vector<bean2d> &Bean_Black, 
        std::vector<bean2d> &Bean_DarkBlue, 
        std::vector<bean2d> &Bean_Gray, 
        std::vector<bean2d> &Bean_Green, 
        std::vector<bean2d> &Bean_LightBlue, 
        std::vector<bean2d> &Bean_Orng, 
        std::vector<bean2d> &Bean_Prpl, 
        std::vector<bean2d> &Bean_Red, 
        std::vector<grasper2d> &Grasper_B,
        std::vector<grasper2d> &Grasper_W);
private:
    // Preprocess the input
    std::vector<std::vector<cv::cuda::GpuMat>> preprocess(const cv::cuda::GpuMat& gpuImg);

    // Postprocess the output
    std::vector<Object> postprocessDetect(std::vector<float>& featureVector);

    // Postprocess the output for pose model
    std::vector<Object> postprocessPose(std::vector<float>& featureVector);

    // Postprocess the output for segmentation model
    std::vector<Object> postProcessSegmentation(std::vector<std::vector<float>>& featureVectors);



    std::unique_ptr<Engine> m_trtEngine = nullptr;

    // Used for image preprocessing
    // YoloV8 model expects values between [0.f, 1.f] so we use the following params
    const std::array<float, 3> SUB_VALS {0.f, 0.f, 0.f};
    const std::array<float, 3> DIV_VALS {1.f, 1.f, 1.f};
    const bool NORMALIZE = true;

    float m_ratio = 1;
    float m_imgWidth = 0;
    float m_imgHeight = 0;

    // Filter thresholds
    const float PROBABILITY_THRESHOLD;
    const float NMS_THRESHOLD;
    const int TOP_K;

    // Segmentation constants
    const int SEG_CHANNELS;
    const int SEG_H;
    const int SEG_W;
    const float SEGMENTATION_THRESHOLD;

    // Object classes as strings
    const std::vector<std::string> CLASS_NAMES;

    // Pose estimation constant
    const int NUM_KPS;
    const float KPS_THRESHOLD;

    // Color list for drawing objects
    const std::vector<std::vector<float>> COLOR_LIST = {
            {1, 1, 1},
            {0.000, 0.500, 0.500}
    };

    const std::vector<std::vector<unsigned int>> KPS_COLORS = {
            {0, 255, 0},
            {255, 128, 0},
            {51, 153, 255},
            {51, 153, 255}
    };

    const std::vector<std::vector<unsigned int>> SKELETON = {
            {0, 1},
            {1, 2},
            {1, 3}
    };

    const std::vector<std::vector<unsigned int>> LIMB_COLORS = {
            {51, 153, 255},
            {51, 153, 255},
            {51, 153, 255},
            {51, 153, 255}
    };
};
