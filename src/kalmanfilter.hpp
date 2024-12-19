// use kalman filter to denoise 2D pose estimation and 3D reconstruction results
// Three types of filters: 1d, 2d, and 3d. 

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>
#include "container.hpp"

struct KalmanFilter1d
{
private:
    int lifeCounter;

public:
    bool KFValid;
    cv::KalmanFilter KF;
    float output;

    KalmanFilter1d(float val);
    void update(float val, bool valid);
    void update();
};

struct KalmanFilter2d
{
private:
    int lifeCounter;

public:
    bool KFValid;
    cv::KalmanFilter KF;
    std::vector<float> output;

    KalmanFilter2d(cv::Point2i point);
    void update(cv::Point2i point, bool detectionValid);
    void update();

};

struct bean2dKF
{
    bool valid;

    KalmanFilter2d beanCenterKF;

    bean2dKF(bean2d bean);
    void update(bean2d bean);
    void update();
};

struct grasper2dKF
{
    bool valid;

    KalmanFilter2d locatorKF;
    KalmanFilter2d jointKF;
    KalmanFilter2d leftFingerKF;
    KalmanFilter2d rightFingerKF;

    grasper2dKF(grasper2d grasper);
    void update(grasper2d grasper);
    void update();
};

class KalmanFilter3d
{
private:
    int lifeCounter;

public:
    bool KFValid;
    cv::KalmanFilter KF;
    std::vector<float> output;

    KalmanFilter3d(cv::Point3f point);
    void update(cv::Point3f point, bool detectionValid);
    void update();
};

struct bean3dKF
{
    bool valid;

    KalmanFilter3d beanCenterKF;

    int born;

    bean3dKF(bean3d bean);
    void update(bean3d bean);
    void update();
};

struct grasper3dKF
{
    bool valid;

    KalmanFilter3d directionKF;
    KalmanFilter3d jointKF;
    KalmanFilter3d locatorKF;
    KalmanFilter3d leftFingerKF;
    KalmanFilter3d rightFingerKF;

    // rotation and angle
    KalmanFilter1d rotation;
    KalmanFilter1d angle;

    int born;

    grasper3dKF(grasper3d grasper);
    void update(grasper3d grasper);
    void update();
};

#endif
