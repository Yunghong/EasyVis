// this module calculates the testing scores using BPE.

#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <numeric>

#include "pose.hpp"
#include "validation.hpp"

Validation::Validation(CameraPose *pose, int views){
    cameraPose=pose;
    viewNumber=views;

    grasperJointCounter=0;
    grasperLocatorCounter=0;
    grasperFingerCounter=0;
    beanCenterCounter=0;
    averageCounter=0;
}

cv::Point2f Validation::backProject(const cv::Matx34d &P, std::vector<float>  &W){

    cv::Point2f u;

    double s=P(2,0)*W[0]+P(2,1)*W[1]+P(2,2)*W[2]+P(2,3);
    
    u.x=(P(0,0)*W[0]+P(0,1)*W[1]+P(0,2)*W[2]+P(0,3))/s;
    u.y=(P(1,0)*W[0]+P(1,1)*W[1]+P(1,2)*W[2]+P(1,3))/s;

    return u;
}

float Validation::getDistance(cv::Point2i groundTruth, cv::Point2f backProjected){
    return sqrt(((float)groundTruth.x-backProjected.x)*((float)groundTruth.x-backProjected.x)
               +((float)groundTruth.y-backProjected.y)*((float)groundTruth.y-backProjected.y));
}

void Validation::BPEAppend(
        std::unordered_map<int, grasper3dKF> &graspersKF3d, 
        std::unordered_map<int, bean3dKF> &beansKF3d,
        std::vector<std::vector<grasper2d>> grasper,
        std::vector<std::vector<bean2d>> bean){

    for(int i=0;i<viewNumber;i++){
        if(!graspersKF3d.empty() && graspersKF3d.at(0).locatorKF.KFValid && !grasper[i].empty() && grasper[i][0].locatorValid){

            cv::Point2f backProjected=backProject(cameraPose->projMatrices[i],graspersKF3d.at(0).locatorKF.output);

            float distance=getDistance(grasper[i][0].locator,backProjected);

            // std::cout<<backProjected<<" "<<distance<<std::endl;

            grasperLocator.push_back(distance);


            grasperLocatorCounter++;


            average.push_back(distance);

            averageCounter++;


        }

        if(!graspersKF3d.empty() && graspersKF3d.at(0).jointKF.KFValid && !grasper[i].empty() && grasper[i][0].jointValid){
            cv::Point2f backProjected=backProject(cameraPose->projMatrices[i],graspersKF3d.at(0).jointKF.output);
            float distance=getDistance(grasper[i][0].joint,backProjected);

            grasperJoint.push_back(distance);
            grasperJointCounter++;

            average.push_back(distance);
            averageCounter++;
        }

        if(!graspersKF3d.empty() && graspersKF3d.at(0).leftFingerKF.KFValid && !grasper[i].empty() && grasper[i][0].leftFingerValid){
            cv::Point2f backProjected=backProject(cameraPose->projMatrices[i],graspersKF3d.at(0).leftFingerKF.output);
            float distance=getDistance(grasper[i][0].leftFinger,backProjected);

            grasperFinger.push_back(distance);
            grasperFingerCounter++;

            average.push_back(distance);
            averageCounter++;
        }

        if(!graspersKF3d.empty() && graspersKF3d.at(0).rightFingerKF.KFValid && !grasper[i].empty() && grasper[i][0].rightFingerValid){
            cv::Point2f backProjected=backProject(cameraPose->projMatrices[i],graspersKF3d.at(0).rightFingerKF.output);
            float distance=getDistance(grasper[i][0].rightFinger,backProjected);

            grasperFinger.push_back(distance);
            grasperFingerCounter++;

            average.push_back(distance);
            averageCounter++;
        }
    }

    for(int i=0;i<viewNumber;i++){
        if(!beansKF3d.empty() && beansKF3d.at(0).beanCenterKF.KFValid && !bean[i].empty() && bean[i][0].beanCenterValid){
            cv::Point2f backProjected=backProject(cameraPose->projMatrices[i],beansKF3d.at(0).beanCenterKF.output);
            float distance=getDistance(bean[i][0].beanCenter,backProjected);

            beanCenter.push_back(distance);
            beanCenterCounter++;

            average.push_back(distance);
            averageCounter++;
        }
    }     

}

float Validation::sumVector(std::vector<float> input){
    float sum=0;

    for(auto v:input){
        sum+=v;
    }

    return sum;
}

// summary final results
void Validation::BPESummary(){
    float totalGrasperLocator = sumVector(grasperLocator)/grasperLocatorCounter;
    float totalGrasperJoint = sumVector(grasperJoint)/grasperJointCounter;
    float totalGrasperFinger = sumVector(grasperFinger)/grasperFingerCounter;
    float totalBeanCenter = sumVector(beanCenter)/beanCenterCounter;
    float totalAverage = sumVector(average)/averageCounter;

    std::cout<<"grasper locator BPE: "<<totalGrasperLocator<<std::endl;
    std::cout<<"grasper joint BPE: "<<totalGrasperJoint<<std::endl;
    std::cout<<"grasper finger BPE: "<<totalGrasperFinger<<std::endl;
    std::cout<<"bean center BPE: "<<totalBeanCenter<<std::endl;
    std::cout<<"grasper avg BPE: "<<(totalGrasperLocator+totalGrasperJoint+totalGrasperFinger)/3<<std::endl;
    std::cout<<"average BPE: "<<totalAverage<<std::endl;
}
