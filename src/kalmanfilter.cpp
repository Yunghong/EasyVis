// use kalman filter to denoise 2D pose estimation and 3D reconstruction results
// Three types of filters: 1d, 2d, and 3d. 

#include "kalmanfilter.hpp"
#include "container.hpp"
#include <iostream>

KalmanFilter1d::KalmanFilter1d(float val): KF(1,1,0){
    KF.transitionMatrix = cv::Mat::eye(1, 1, CV_32F);
    KF.measurementMatrix = cv::Mat::eye(1, 1, CV_32F);
    KF.processNoiseCov = cv::Mat::eye(1, 1, CV_32F) * 0.1;
    KF.measurementNoiseCov = cv::Mat::eye(1, 1, CV_32F) * 0.1;
    KF.errorCovPost = cv::Mat::eye(1, 1, CV_32F);

    KF.statePost.at<float>(0) = val;

    lifeCounter=0;
    KFValid=true;
}

KalmanFilter2d::KalmanFilter2d(cv::Point2i point): KF(2,2,0){
    KF.transitionMatrix = cv::Mat::eye(2, 2, CV_32F);
    KF.measurementMatrix = cv::Mat::eye(2, 2, CV_32F);
    KF.processNoiseCov = cv::Mat::eye(2, 2, CV_32F) * 0.1;
    KF.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * 0.1;
    KF.errorCovPost = cv::Mat::eye(2, 2, CV_32F);

    KF.statePost.at<float>(0) = float(point.x);
    KF.statePost.at<float>(1) = float(point.y);

    lifeCounter=5;
    KFValid=true;
    output.resize(2);
}

KalmanFilter3d::KalmanFilter3d(cv::Point3f point): KF(3,3,0){
    KF.transitionMatrix = cv::Mat::eye(3, 3, CV_32F);
    KF.measurementMatrix = cv::Mat::eye(3, 3, CV_32F);
    KF.processNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 0.01;
    KF.measurementNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 0.01;
    KF.errorCovPost = cv::Mat::eye(3, 3, CV_32F)*0.01;

    KF.statePost.at<float>(0) = float(point.x);
    KF.statePost.at<float>(1) = float(point.y);
    KF.statePost.at<float>(2) = float(point.z);

    lifeCounter=0;
    KFValid=true;
    output.resize(3);
}

void KalmanFilter1d::update(float val, bool valid){
    // if long time not detecting the key point, kill this kalman object
    if(lifeCounter<0){
        KFValid=false;
        return;
    }

    if(valid){
        lifeCounter=0;
        
        if(abs(val-KF.statePost.at<float>(0))>0.8*M_PI){
            output=val;
            KF.statePost.at<float>(0)=val;
            KF.predict();
        }
        else{
            cv::Mat measurement = (cv::Mat_<float>(1, 1) << val);     
            cv::Mat correction = KF.correct(measurement);   
            KF.predict();
            output=correction.at<float>(0);
        }
        

    }
    else{
        lifeCounter--;
        cv::Mat prediction = KF.predict();
        output=prediction.at<float>(0);
    }
}

void KalmanFilter1d::update(){
    // if long time not detecting the key point, kill this kalman object
    if(lifeCounter<0){
        KFValid=false;
        return;
    }

    lifeCounter--;
    cv::Mat prediction = KF.predict();
    output=prediction.at<float>(0);
}

void KalmanFilter2d::update(cv::Point2i point, bool detectionValid){
    // if long time not detecting the key point, kill this kalman object
    if(lifeCounter<0){
        KFValid=false;
        return;
    }

    if(detectionValid){
        lifeCounter=5;
        
        cv::Mat measurement = (cv::Mat_<float>(2, 1) << float(point.x), float(point.y));     

        cv::Mat correction = KF.correct(measurement);   
        KF.predict();
        output[0]=correction.at<float>(0);
        output[1]=correction.at<float>(1);
    }
    else{
        lifeCounter--;
        cv::Mat prediction = KF.predict();
        output[0]=prediction.at<float>(0);
        output[1]=prediction.at<float>(1);
    }
}

void KalmanFilter3d::update(cv::Point3f point, bool reconstructValid){
    // if long time not detecting the key point, kill this kalman object
    if(lifeCounter<0){
        KFValid=false;
        return;
    }

    if(reconstructValid){
        lifeCounter=0;
        
        cv::Mat measurement = (cv::Mat_<float>(3, 1) << float(point.x), float(point.y), float(point.z));     

        cv::Mat correction = KF.correct(measurement);   
        KF.predict();
        output[0]=correction.at<float>(0);
        output[1]=correction.at<float>(1);
        output[2]=correction.at<float>(2);
    }
    else{
        lifeCounter--;
        cv::Mat prediction = KF.predict();
        output[0]=prediction.at<float>(0);
        output[1]=prediction.at<float>(1);
        output[2]=prediction.at<float>(2);
    }
}

void KalmanFilter2d::update(){
    // if long time not detecting the key point, kill this kalman object
    if(lifeCounter<0){
        KFValid=false;
        return;
    }

    lifeCounter--;
    cv::Mat prediction = KF.predict();
    output[0]=prediction.at<float>(0);
    output[1]=prediction.at<float>(1);
}

void KalmanFilter3d::update(){
    // if long time not detecting the key point, kill this kalman object
    if(lifeCounter<0){
        KFValid=false;
        return;
    }

    lifeCounter--;
    cv::Mat prediction = KF.predict();
    output[0]=prediction.at<float>(0);
    output[1]=prediction.at<float>(1);
    output[2]=prediction.at<float>(2);

}

grasper2dKF::grasper2dKF(grasper2d grasper):
    locatorKF(grasper.locator),
    jointKF(grasper.joint),
    leftFingerKF(grasper.leftFinger),
    rightFingerKF(grasper.rightFinger){

    valid=true;
}

void grasper2dKF::update(grasper2d grasper){  
    locatorKF.update(grasper.locator,grasper.locatorValid);
    jointKF.update(grasper.joint,grasper.jointValid);
    leftFingerKF.update(grasper.leftFinger,grasper.leftFingerValid);
    rightFingerKF.update(grasper.rightFinger,grasper.rightFingerValid);

    if(!locatorKF.KFValid || !jointKF.KFValid || (!leftFingerKF.KFValid && rightFingerKF.KFValid)){
        valid=false;
    }
    else{
        valid=true;
    }
}

void grasper2dKF::update(){  
    locatorKF.update();
    jointKF.update();
    leftFingerKF.update();
    rightFingerKF.update();

    if(!locatorKF.KFValid || !jointKF.KFValid || (!leftFingerKF.KFValid && rightFingerKF.KFValid)){
        valid=false;
    }
    else{
        valid=true;
    }
}

bean2dKF::bean2dKF(bean2d bean):
    beanCenterKF(bean.beanCenter){

    valid=true;
}

void bean2dKF::update(bean2d bean){  
    beanCenterKF.update(bean.beanCenter,bean.beanCenterValid);

    valid=beanCenterKF.KFValid;
}

void bean2dKF::update(){  
    beanCenterKF.update();

    valid=beanCenterKF.KFValid;
}

grasper3dKF::grasper3dKF(grasper3d grasper):
    directionKF(grasper.direction),
    jointKF(grasper.joint),
    locatorKF(grasper.locator),
    leftFingerKF(grasper.leftFinger),
    rightFingerKF(grasper.rightFinger),
    rotation(grasper.rotation),
    angle(grasper.angle){

    for(int i=0;i<5;i++){
        directionKF.update(grasper.direction, true);
        jointKF.update(grasper.joint, true);
        locatorKF.update(grasper.locator, true);
        leftFingerKF.update(grasper.leftFinger, true);
        rightFingerKF.update(grasper.rightFinger, true);
        rotation.update(grasper.rotation, true);
        angle.update(grasper.angle, true);
    }

    valid=true;
}

void grasper3dKF::update(grasper3d grasper){  
    directionKF.update(grasper.direction, true);
    jointKF.update(grasper.joint, true);
    locatorKF.update(grasper.locator, true);
    leftFingerKF.update(grasper.leftFinger, true);
    rightFingerKF.update(grasper.rightFinger, true);
    rotation.update(grasper.rotation, true);
    angle.update(grasper.angle, true);

    valid=true;
}

void grasper3dKF::update(){  
    directionKF.update();
    jointKF.update();
    rotation.update();
    angle.update();

    valid=true;
}

bean3dKF::bean3dKF(bean3d bean):
    beanCenterKF(bean.beanCenter){

    //dirty fix, fix error position in first few run
    for(int i=0;i<5;i++) beanCenterKF.update(bean.beanCenter,bean.beanCenterValid);
}

void bean3dKF::update(bean3d bean){  
    beanCenterKF.update(bean.beanCenter,bean.beanCenterValid);

    valid=beanCenterKF.KFValid;
}

void bean3dKF::update(){  
    beanCenterKF.update();

    valid=beanCenterKF.KFValid;
}
