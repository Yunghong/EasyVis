#ifndef CONTAINER_H
#define CONTAINER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// our basic container, a kind of data type to store grasper and bean states

struct bean2d{
    cv::Point2i beanCenter;
    cv::Rect_<float> bbox;

    bool beanCenterValid;
};

struct grasper2d{
    cv::Point2i locator;
    cv::Point2i joint;
    cv::Point2i leftFinger;
    cv::Point2i rightFinger;
    cv::Rect_<float> bbox;

    bool locatorValid;
    bool jointValid;
    bool leftFingerValid;
    bool rightFingerValid;
};

struct bean3d{
    cv::Point3f beanCenter;

    bool beanCenterValid;
};

struct grasper3d{
    cv::Point3f direction;
    cv::Point3f locator;
    cv::Point3f joint;
    cv::Point3f leftFinger;
    cv::Point3f rightFinger;
    float rotation;
    float angle;

    bool locatorValid;
    bool jointValid;
    bool leftFingerValid;
    bool rightFingerValid;

    //bool valid;
};
#endif
