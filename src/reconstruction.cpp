// this module handles 3D reconstruction and 3D pose estimation

#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "reconstruction.hpp"
#include "container.hpp"
#include "kalmanfilter.hpp"
#include "config.hpp"
#include "pose.hpp"
#include "BYTETracker.h"


// init reconstruction module
Reconstructor::Reconstructor(EasyVisConfig *config, CameraPose *cameraPose){
    graspers_B_Paired.resize(config->VIEW_NUMBER);
    graspers_W_Paired.resize(config->VIEW_NUMBER);

    beans_blk_Paired.resize(config->VIEW_NUMBER);
    beans_dkbl_Paired.resize(config->VIEW_NUMBER);
    beans_gray_Paired.resize(config->VIEW_NUMBER);
    beans_gren_Paired.resize(config->VIEW_NUMBER);
    beans_ltbl_Paired.resize(config->VIEW_NUMBER);
    beans_orng_Paired.resize(config->VIEW_NUMBER);
    beans_prpl_Paired.resize(config->VIEW_NUMBER);
    beans_red_Paired.resize(config->VIEW_NUMBER);

    graspers_B_KF.resize(config->VIEW_NUMBER);
    graspers_W_KF.resize(config->VIEW_NUMBER);

    beans_blk_KF.resize(config->VIEW_NUMBER);
    beans_dkbl_KF.resize(config->VIEW_NUMBER);
    beans_gray_KF.resize(config->VIEW_NUMBER);
    beans_gren_KF.resize(config->VIEW_NUMBER);
    beans_ltbl_KF.resize(config->VIEW_NUMBER);
    beans_orng_KF.resize(config->VIEW_NUMBER);
    beans_prpl_KF.resize(config->VIEW_NUMBER);
    beans_red_KF.resize(config->VIEW_NUMBER);

    viewNumber=config->VIEW_NUMBER;
    pose=cameraPose;
    grasperIdStart=0;
    beanIdStart=0;
}

// cal dist between two point
int Reconstructor::getDistance(cv::Point2i X0, cv::Point2i X1){
    return sqrt((X0.x-X1.x)*(X0.x-X1.x)+(X0.y-X1.y)*(X0.y-X1.y));
}

// estimate number of instances from multi-camera array
template<typename T>
int Reconstructor::getObjectNumber(std::vector<std::vector<T>> objects){
    std::unordered_map<int, int> count_map;

    // Count occurrences of each number
    for(auto &object:objects){
        count_map[int(object.size())]++;
    }

    // Find the number with the maximum count
    int max_count = 0;
    int most_common_number = 0;
    for (const auto& pair : count_map) {
        if (pair.second > max_count) {
            max_count = pair.second;
            most_common_number = pair.first;
        }
    }

    return most_common_number;
}


// cal dist from two points
double Reconstructor::calculateDistance(double x1, double y1, double x2, double y2) {
    // Compute the differences in coordinates
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    // Calculate the Euclidean distance
    return std::sqrt(dx * dx + dy * dy);
}

// prepare/arrange multi-view 2D objects for 3D reconstruction 
void Reconstructor::prepareSingleObject_bean(
        std::vector<std::vector<bean2d>> &beans,
        std::vector<std::unordered_map<int, bean2d>>& beansPaired,
        std::vector<std::unordered_map<int, bean2dKF>>& beansKF,
        std::unordered_set<int>& beanIds){

    std::vector<int> beanValidView=getValidViewNumber(beans);

    for(auto viewIdx:beanValidView){
        beansPaired[viewIdx][0]=beans[viewIdx][0];
        if(beansPaired[viewIdx].empty()){
            // by pass pair, grasperPaired[idx] is the hash
            beansPaired[viewIdx][0]=beans[viewIdx][0];
        }       
        if(beansKF[viewIdx].empty()){
            beansKF[viewIdx].insert({0, bean2dKF(beans[viewIdx][0])});
            if(beanIds.find(0)== beanIds.end()) beanIds.insert(0);
        }
    }

    int validViewCounterBean=0;
    for(int i=0;i<viewNumber;i++){
    // update bean KFs
        if(!beansKF[i].empty() && beansKF[i].at(0).valid){
            if(!beans[i].empty()){
                beansKF[i].at(0).update(beans[i][0]);
            }
            else{
                beansKF[i].at(0).update();
            }

            validViewCounterBean++;
        }
        else{
            beansPaired[i].clear();
            beansKF[i].clear();
        }
    
    }

    if(validViewCounterBean<2){
        beanIds.erase(0);
        for(int i=0;i<viewNumber;i++){
            beansPaired[i].clear();
            beansKF[i].clear();
        }
    }
}

//reconstruct grasper, when only one grasper in the system
void Reconstructor::prepareSingleObject_grasper(
        std::vector<std::vector<grasper2d>> &graspers,
        std::vector<std::unordered_map<int, grasper2d>>& graspersPaired,
        std::vector<std::unordered_map<int, grasper2dKF>>& graspersKF,
        std::unordered_set<int>& grasperIds){

    std::vector<int> grasperValidView=getValidViewNumber(graspers);

    for(auto idx:grasperValidView){
        graspersPaired[idx][0]=graspers[idx][0];
        if(graspersPaired[idx].empty()){
            // by pass pair, grasperPaired[idx] is the hash
            graspersPaired[idx][0]=graspers[idx][0];
        }       
        if(graspersKF[idx].empty()){
            // graspersKF[idx].emplace(0, grasper2dKF(graspers[idx][0]));
            graspersKF[idx].insert({0, grasper2dKF(graspers[idx][0])});
            if(grasperIds.find(0)== grasperIds.end()){
                grasperIds.insert(0);
            }
        }
    }

    int validViewCounterGrasper=0;

    for(int i=0;i<viewNumber;i++){
        // update grasper KFs
        if(!graspersKF[i].empty() && graspersKF[i].at(0).valid){
            if(!graspers[i].empty()){
                graspersKF[i].at(0).update(graspers[i][0]);
            }
            else{
                graspersKF[i].at(0).update();
            }
            validViewCounterGrasper++;
        }
        else{
            graspersPaired[i].clear();
            graspersKF[i].clear();
        }
    }

    if(validViewCounterGrasper<2){
        grasperIds.erase(0);
        for(int i=0;i<viewNumber;i++){
            graspersPaired[i].clear();
            graspersKF[i].clear();
        }
    }
}

// prepare/arrange multi-view 2D objects for 3D reconstruction 
void Reconstructor::prepareSingleObject(
        std::vector<std::vector<bean2d>> Bean_Black,
        std::vector<std::vector<bean2d>> Bean_DarkBlue,
        std::vector<std::vector<bean2d>> Bean_Gray,
        std::vector<std::vector<bean2d>> Bean_Green,
        std::vector<std::vector<bean2d>> Bean_LightBlue,
        std::vector<std::vector<bean2d>> Bean_Orng,
        std::vector<std::vector<bean2d>> Bean_Prpl,
        std::vector<std::vector<bean2d>> Bean_Red,
        std::vector<std::vector<grasper2d>> Grasper_B,
        std::vector<std::vector<grasper2d>> Grasper_W){

    prepareSingleObject_bean(Bean_Black, beans_blk_Paired,beans_blk_KF,bean_blk_Ids);
    prepareSingleObject_bean(Bean_DarkBlue, beans_dkbl_Paired,beans_dkbl_KF,bean_dkbl_Ids);
    prepareSingleObject_bean(Bean_Gray, beans_gray_Paired,beans_gray_KF,bean_gray_Ids);
    prepareSingleObject_bean(Bean_Green, beans_gren_Paired,beans_gren_KF,bean_gren_Ids);
    prepareSingleObject_bean(Bean_LightBlue, beans_ltbl_Paired,beans_ltbl_KF,bean_ltbl_Ids);
    prepareSingleObject_bean(Bean_Orng, beans_orng_Paired,beans_orng_KF,bean_orng_Ids);
    prepareSingleObject_bean(Bean_Prpl, beans_prpl_Paired,beans_prpl_KF,bean_prpl_Ids);
    prepareSingleObject_bean(Bean_Red, beans_red_Paired,beans_red_KF,bean_red_Ids);

    prepareSingleObject_grasper(Grasper_B,graspers_B_Paired,graspers_B_KF,grasper_B_Ids);
    prepareSingleObject_grasper(Grasper_W,graspers_W_Paired,graspers_W_KF,grasper_W_Ids);
}

// check how many view valid
template<typename T>
std::vector<int> Reconstructor::getValidViewNumber(std::vector<std::vector<T>> objectVector){
    std::vector<int> validViews;

    for(int i=0;i<objectVector.size();i++){
        if(!objectVector[i].empty())
            validViews.push_back(i);

    }

    return validViews;
}

// run reconstruction
void Reconstructor::run(
        std::vector<std::vector<bean2d>> Bean_Black,
        std::vector<std::vector<bean2d>> Bean_DarkBlue,
        std::vector<std::vector<bean2d>> Bean_Gray,
        std::vector<std::vector<bean2d>> Bean_Green,
        std::vector<std::vector<bean2d>> Bean_LightBlue,
        std::vector<std::vector<bean2d>> Bean_Orng,
        std::vector<std::vector<bean2d>> Bean_Prpl,
        std::vector<std::vector<bean2d>> Bean_Red,
        std::vector<std::vector<grasper2d>> Grasper_B,
        std::vector<std::vector<grasper2d>> Grasper_W){

    prepareSingleObject(
            Bean_Black,
            Bean_DarkBlue,
            Bean_Gray,
            Bean_Green,
            Bean_LightBlue,
            Bean_Orng,
            Bean_Prpl,
            Bean_Red,
            Grasper_B,
            Grasper_W);

    reconstructSingleObject();
}

// bean reconstruction
void Reconstructor::reconstructSingleObject_bean(
        std::unordered_set<int>& beanIds,
        std::vector<std::unordered_map<int, bean2d>>& beansPaired,
        std::vector<std::unordered_map<int, bean2dKF>>& beansKF,
        std::unordered_map<int, bean3d>& beans3d,
        std::unordered_map<int, bean3dKF>& beansKF3d){
    // bean
    beans3d.clear();

    for(auto id:beanIds){
        std::vector<cv::Mat> pj;
        std::vector<cv::Mat> beanCenter2d;
        cv::Mat beanCenter3dMat;

        for(int view=0;view<viewNumber;view++){
            if(!beansKF[view].empty() && beansPaired[view][id].beanCenterValid){
                pj.push_back(pose->projMatrices[view]);
                beanCenter2d.push_back((cv::Mat_<double>(2,1) << double(beansPaired[view][id].beanCenter.x), double(beansPaired[view][id].beanCenter.y)));
            }
        }
        
        normalized_triangulatePoints(pj, beanCenter2d, beanCenter3dMat, 480, 640); 

        bean3d bean3dTmp;

        if(pj.size()>=2){
            bean3dTmp.beanCenterValid=true;

            bean3dTmp.beanCenter.x=beanCenter3dMat.at<double>(0);
            bean3dTmp.beanCenter.y=beanCenter3dMat.at<double>(1);
            bean3dTmp.beanCenter.z=beanCenter3dMat.at<double>(2);
        }
        else{
            beansKF3d.erase(id);
        }

        if(pj.size()<2){
            beansKF3d.erase(id);
        }
        else{
            beans3d.insert({id, bean3dTmp});
        }
        

        if(pj.size()>=2){
            if(beansKF3d.find(id)== beansKF3d.end()){
                beansKF3d.insert({id,bean3dKF(bean3dTmp)});
            }
            else{
                beansKF3d.at(id).update(bean3dTmp);
            }
        }
    }

    if(beanIds.empty()){
        beansKF3d.clear();
    }
    else{
        for(auto id:beanIds){
            if(beansKF3d.find(id)==beansKF3d.end()){
                beansKF3d.erase(id);
            }
        }
    }
}

// grasper reconstruction
void Reconstructor::reconstructSingleObject_grasper(
        std::unordered_set<int>& grasperIds,
        std::vector<std::unordered_map<int, grasper2d>>& graspersPaired,
        std::vector<std::unordered_map<int, grasper2dKF>>& graspersKF,
        std::unordered_map<int, grasper3d>& graspers3d,
        std::unordered_map<int, grasper3dKF>& graspersKF3d){
    graspers3d.clear();

    // grasper
    for(auto id:grasperIds){
        // std::cout<<"id "<<id<<std::endl;
        std::vector<cv::Mat> pj,pjLeft,pjRight;
        std::vector<cv::Mat> joint2d,locator2d,leftFinger2d,rightFinger2d;
        cv::Mat joint3dMat,locator3dMat,leftFinger3dMat,rightFinger3dMat;

        for(int view=0;view<viewNumber;view++){
            if(!graspersPaired[view].empty() && graspersPaired[view][id].jointValid && graspersPaired[view][id].locatorValid){
                pj.push_back(pose->projMatrices[view]);
                joint2d.push_back((cv::Mat_<double>(2,1) << double(graspersPaired[view][id].joint.x), double(graspersPaired[view][id].joint.y)));
                locator2d.push_back((cv::Mat_<double>(2,1) << double(graspersPaired[view][id].locator.x), double(graspersPaired[view][id].locator.y)));
            }

            if(!graspersPaired[view].empty() && graspersPaired[view][id].jointValid && graspersPaired[view][id].locatorValid && graspersPaired[view][id].leftFingerValid ){
                pjLeft.push_back(pose->projMatrices[view]);
                leftFinger2d.push_back((cv::Mat_<double>(2,1) << double(graspersPaired[view][id].leftFinger.x), double(graspersPaired[view][id].leftFinger.y)));            
            }

            if(!graspersPaired[view].empty() && graspersPaired[view][id].jointValid && graspersPaired[view][id].locatorValid && graspersPaired[view][id].rightFingerValid){
                pjRight.push_back(pose->projMatrices[view]);
                rightFinger2d.push_back((cv::Mat_<double>(2,1) << double(graspersPaired[view][id].rightFinger.x), double(graspersPaired[view][id].rightFinger.y)));            
            }
        }
        
        normalized_triangulatePoints(pj, joint2d, joint3dMat, 480, 640); 
        normalized_triangulatePoints(pj, locator2d, locator3dMat, 480, 640); 
        normalized_triangulatePoints(pjLeft, leftFinger2d, leftFinger3dMat, 480, 640); 
        normalized_triangulatePoints(pjRight, rightFinger2d, rightFinger3dMat, 480, 640); 

        grasper3d grasper3dTmp;
        grasper3dTmp.leftFingerValid=false;
        grasper3dTmp.rightFingerValid=false;

        if(pj.size()>=2){
            grasper3dTmp.jointValid=true;
            grasper3dTmp.locatorValid=true;

            grasper3dTmp.joint.x=joint3dMat.at<double>(0);
            grasper3dTmp.joint.y=joint3dMat.at<double>(1);
            grasper3dTmp.joint.z=joint3dMat.at<double>(2);

            grasper3dTmp.locator.x=locator3dMat.at<double>(0);
            grasper3dTmp.locator.y=locator3dMat.at<double>(1);
            grasper3dTmp.locator.z=locator3dMat.at<double>(2);

            grasper3dTmp.direction.x=grasper3dTmp.locator.x-grasper3dTmp.joint.x;
            grasper3dTmp.direction.y=grasper3dTmp.locator.y-grasper3dTmp.joint.y;
            grasper3dTmp.direction.z=grasper3dTmp.locator.z-grasper3dTmp.joint.z;

            std::cout<<"here"<<std::endl;
        }
        else{
            graspersKF3d.erase(id);
            //std::cout<<"erase grasper"<<std::endl;
        }

        if(pj.size()>=2 && pjLeft.size()>=2){
            grasper3dTmp.leftFingerValid=true;

            grasper3dTmp.leftFinger.x=leftFinger3dMat.at<double>(0);
            grasper3dTmp.leftFinger.y=leftFinger3dMat.at<double>(1);
            grasper3dTmp.leftFinger.z=leftFinger3dMat.at<double>(2);
        }
        else{
            graspersKF3d.erase(id);
            //std::cout<<"erase grasper"<<std::endl;
        }

        if(pj.size()>=2 && pjRight.size()>=2){
            grasper3dTmp.rightFingerValid=true;

            grasper3dTmp.rightFinger.x=rightFinger3dMat.at<double>(0);
            grasper3dTmp.rightFinger.y=rightFinger3dMat.at<double>(1);
            grasper3dTmp.rightFinger.z=rightFinger3dMat.at<double>(2);
        }
        else{
            graspersKF3d.erase(id);
            //std::cout<<"erase grasper"<<std::endl;
        }

        reconstructFingerPose(pjLeft.size(), pjRight.size(), grasper3dTmp);

        if(pj.size()<2 || (pjRight.size()<2 && pjLeft.size()<2) ){
            graspersKF3d.erase(id);
        }
        else{
            graspers3d.insert({id, grasper3dTmp});
        }

        if(pj.size()>=2 && (pjRight.size()>=2 || pjLeft.size()>=2)){
            if(graspersKF3d.find(id)== graspersKF3d.end()){
                graspersKF3d.insert({id,grasper3dKF(grasper3dTmp)});
            }
            else{
                graspersKF3d.at(id).update(grasper3dTmp);
            }
        }
        

        // if(graspersKF3d.find(id)== graspersKF3d.end()){
        //     graspersKF3d.insert({id,grasper3dKF(grasper3dTmp)});
        // }
        // else{
        //     graspersKF3d.at(id).update(grasper3dTmp);
        // }

    }

    if(grasperIds.empty()){
        graspersKF3d.clear();
    }
    else{
        for(auto id:grasperIds){
            if(graspersKF3d.find(id)==graspersKF3d.end()){
                graspersKF3d.erase(id);
            }
        }
    }   
}

// 3D reconstruction
void Reconstructor::reconstructSingleObject(){
    reconstructSingleObject_bean(bean_blk_Ids,beans_blk_Paired,beans_blk_KF,beans_blk_3d,beans_blk_KF3d);
    reconstructSingleObject_bean(bean_dkbl_Ids,beans_dkbl_Paired,beans_dkbl_KF,beans_dkbl_3d,beans_dkbl_KF3d);
    reconstructSingleObject_bean(bean_gray_Ids,beans_gray_Paired,beans_gray_KF,beans_gray_3d,beans_gray_KF3d);
    reconstructSingleObject_bean(bean_gren_Ids,beans_gren_Paired,beans_gren_KF,beans_gren_3d,beans_gren_KF3d);
    reconstructSingleObject_bean(bean_ltbl_Ids,beans_ltbl_Paired,beans_ltbl_KF,beans_ltbl_3d,beans_ltbl_KF3d);
    reconstructSingleObject_bean(bean_orng_Ids,beans_orng_Paired,beans_orng_KF,beans_orng_3d,beans_orng_KF3d);
    reconstructSingleObject_bean(bean_prpl_Ids,beans_prpl_Paired,beans_prpl_KF,beans_prpl_3d,beans_prpl_KF3d);
    reconstructSingleObject_bean(bean_red_Ids,beans_red_Paired,beans_red_KF,beans_red_3d,beans_red_KF3d);

    reconstructSingleObject_grasper(grasper_B_Ids,graspers_B_Paired,graspers_B_KF,graspers_B_3d,graspers_B_KF3d);
    reconstructSingleObject_grasper(grasper_W_Ids,graspers_W_Paired,graspers_W_KF,graspers_W_3d,graspers_W_KF3d);
}

// cal Det
float Reconstructor::getDeterminant(const std::vector<std::vector<float>> vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 
    int dimension = vect.size();

    if(dimension == 0) {
        return 1;
    }

    if(dimension == 1) {
        return vect[0][0];
    }

    //Formula for 2x2-matrix
    if(dimension == 2) {
        return vect[0][0] * vect[1][1] - vect[0][1] * vect[1][0];
    }

    float result = 0;
    int sign = 1;
    for(int i = 0; i < dimension; i++) {

        //Submatrix
        std::vector<std::vector<float>> subVect(dimension - 1, std::vector<float> (dimension - 1));
        for(int m = 1; m < dimension; m++) {
            int z = 0;
            for(int n = 0; n < dimension; n++) {
                if(n != i) {
                    subVect[m-1][z] = vect[m][n];
                    z++;
                }
            }
        }

        //recursive call
        result = result + sign * vect[0][i] * getDeterminant(subVect);
        sign = -sign;
    }

    return result;
}

// transpose
std::vector<std::vector<float>> Reconstructor::getTranspose(const std::vector<std::vector<float>> matrix1) {

    //Transpose-matrix: height = width(matrix), width = height(matrix)
    std::vector<std::vector<float>> solution(matrix1[0].size(), std::vector<float> (matrix1.size()));

    //Filling solution-matrix
    for(size_t i = 0; i < matrix1.size(); i++) {
        for(size_t j = 0; j < matrix1[0].size(); j++) {
            solution[j][i] = matrix1[i][j];
        }
    }
    return solution;
}

// inverse
std::vector<std::vector<float>> Reconstructor::getInverse(const std::vector<std::vector<float>> vect) {
    if(getDeterminant(vect) == 0) {
        throw std::runtime_error("Determinant is 0");
    } 

    float d = 1.0/getDeterminant(vect);
    std::vector<std::vector<float>> solution(vect.size(), std::vector<float> (vect.size()));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] = vect[i][j]; 
        }
    }

    solution = getTranspose(getCofactor(solution));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] *= d;
        }
    }

    return solution;
}

std::vector<std::vector<float>> Reconstructor::getCofactor(const std::vector<std::vector<float>> vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 

    std::vector<std::vector<float>> solution(vect.size(), std::vector<float> (vect.size()));
    std::vector<std::vector<float>> subVect(vect.size() - 1, std::vector<float> (vect.size() - 1));

    for(std::size_t i = 0; i < vect.size(); i++) {
        for(std::size_t j = 0; j < vect[0].size(); j++) {

            int p = 0;
            for(size_t x = 0; x < vect.size(); x++) {
                if(x == i) {
                    continue;
                }
                int q = 0;

                for(size_t y = 0; y < vect.size(); y++) {
                    if(y == j) {
                        continue;
                    }

                    subVect[p][q] = vect[x][y];
                    q++;
                }
                p++;
            }
            solution[i][j] = pow(-1, i + j) * getDeterminant(subVect);
        }
    }
    return solution;
}

// (\hat(P)-T)R^-1=R_z*P_0
// solve the equation to get R_z, then get \theta
// P_0 is the moved prebuild finger with calculated angle
void Reconstructor::getFingerRotation(
        std::vector<float> rodDirection, 
        std::vector<float> rodTranslation, 
        std::vector<float> fingerObserved, 
        float angle, float &rotation){

    float d1 = rodDirection[0], d2 = rodDirection[1], d3 = rodDirection[2];
    float alpha;
    std::vector<float> azel(2, 0);
    if ( d1 == 0 && d2 == 0 ){
        azel[0] = 0;
        azel[1] = 0;
    } 
    else if ( (d1 < 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 > 0 && d3 < 0) ){

        azel[0] = atan( d2 / d1 ) + 1.5*M_PI;
        azel[1] = 0;
    }
    else{
        azel[0] = atan( d2 / d1 ) + 0.5*M_PI;
        azel[1] = 0;   
    }

    if ( d1 == 0 && d2 == 0 ){
        alpha = 0;
    } 
    else if ( (d1 < 0 && d2 < 0 && d3 < 0) || (d1 < 0 && d2 < 0 && d3 > 0) ){
        alpha = 1.5*M_PI + atan(d3 / sqrt(d1 * d1 + d2 * d2));
    }
    else{
        alpha = 0.5*M_PI - atan(d3 / sqrt(d1 * d1 + d2 * d2));
    }

    float theta = azel[0];
    float phi = azel[1];
    float x = cos(phi) * cos(theta);
    float y = cos(phi) * sin(theta);
    float z = sin(phi);

    float alph = alpha;
    float cosa = cos(alph);
    float sina = sin(alph);
    float vera = 1 - cosa;

    std::vector<std::vector<float>> rot = {    {cosa + x * x * vera, x * y * vera + z * sina, x * z *vera - y * sina},
                                                {x * y * vera - z * sina, cosa + y * y * vera, y * z * vera + x * sina},
                                                {x * z * vera + y * sina, y * z * vera - x *sina, cosa + z * z * vera} 
                                                };


    std::vector<std::vector<float>> rotInv=getInverse(rot);

    fingerObserved[0]-=rodTranslation[0];
    fingerObserved[1]-=rodTranslation[1];
    fingerObserved[2]-=rodTranslation[2];

    std::vector<float> pTmp={   fingerObserved[0]*rotInv[0][0]+fingerObserved[1]*rotInv[1][0]+fingerObserved[2]*rotInv[2][0],
                                fingerObserved[0]*rotInv[0][1]+fingerObserved[1]*rotInv[1][1]+fingerObserved[2]*rotInv[2][1],
                                fingerObserved[0]*rotInv[0][2]+fingerObserved[1]*rotInv[1][2]+fingerObserved[2]*rotInv[2][2]};

    float fingerLengthSquare=pTmp[0]*pTmp[0]+pTmp[1]*pTmp[1]+pTmp[2]*pTmp[2];
    std::vector<float> P0={0.0,sqrt(fingerLengthSquare-pTmp[2]*pTmp[2]),pTmp[2]};

    float xn=pTmp[1]/P0[1];
    float yn=pTmp[0]/P0[1];

    xn=(xn>1.0)?1.0:xn;
    xn=(xn<-1.0)?-1.0:xn;
    yn=(yn>1.0)?1.0:yn;
    yn=(yn<-1.0)?-1.0:yn;

    float rawRotationX=acos(xn);
    float rawRotationY=asin(yn);


    if(rawRotationX>0.5*M_PI  && rawRotationY>0){
        rawRotationY=rawRotationX;
    }
    else if(rawRotationX>0.5*M_PI && rawRotationY<0){
        rawRotationX=2*M_PI-rawRotationX;
        rawRotationY=rawRotationX;
    }
    else if(rawRotationX<0.5*M_PI && rawRotationY>0){
        // do nothing
    }
    else if(rawRotationX<0.5*M_PI && rawRotationY<0){
        rawRotationX=rawRotationY;
    }

    rotation=(rawRotationX+rawRotationY)/2;

    rotation=(rotation>M_PI)?rotation-M_PI:rotation;
}

void Reconstructor::getFingerRotation(std::vector<float> fingerDirection, float &rotation){
    float u=sqrt(fingerDirection[2]*fingerDirection[2]+fingerDirection[1]*fingerDirection[1]+fingerDirection[0]*fingerDirection[0]);

    float v1=fingerDirection[2]/u;
    float v2=fingerDirection[1]/u;

    rotation=-atan(v1/v2)+1.3;
}

// cal grasper tips open angle
void Reconstructor::getFingerAngle(std::vector<float> d1, std::vector<float> d2, float &angle){
    float u=d1[0]*d2[0]+d1[1]*d2[1]+d1[2]*d2[2];

    float v1=sqrt(d1[0]*d1[0]+d1[1]*d1[1]+d1[2]*d1[2]);
    float v2=sqrt(d2[0]*d2[0]+d2[1]*d2[1]+d2[2]*d2[2]);

    angle=acos(u/(v1*v2));

    angle=(0.25*M_PI*(angle-0.08))/(0.25*M_PI-0.08);
    angle=(angle>0.0)?angle:0.0;
    angle=(angle<0.5*M_PI)?angle:0.5*M_PI;
  
}

// cal grasper 3D pose
void Reconstructor::reconstructFingerPose(size_t pjLeftSize, size_t pjRightSize, grasper3d &grasper){
    if(pjLeftSize>=2 && pjRightSize<2){
        std::vector<float> leftFingerDirection{ grasper.joint.x-grasper.leftFinger.x,
                                                grasper.joint.y-grasper.leftFinger.y,
                                                grasper.joint.z-grasper.leftFinger.z};

        std::vector<float> rodDirection{    grasper.direction.x,
                                            grasper.direction.y,
                                            grasper.direction.z};
                                            
        float u=sqrt(   leftFingerDirection[0]*leftFingerDirection[0]
                       +leftFingerDirection[1]*leftFingerDirection[1]
                       +leftFingerDirection[2]*leftFingerDirection[2]);
                        
        for(auto &val:leftFingerDirection) val/=u;
        
        std::vector<float> rodTranslation={grasper.joint.x,grasper.joint.y,grasper.joint.z};
        std::vector<float> fingerLeftObserved={grasper.leftFinger.x,grasper.leftFinger.y,grasper.leftFinger.z};

        getFingerAngle(leftFingerDirection, rodDirection, grasper.angle); 

        getFingerRotation(rodDirection,rodTranslation,fingerLeftObserved,grasper.angle,grasper.rotation);
    }
    else if(pjLeftSize<2 && pjRightSize>=2){
        std::vector<float> rightFingerDirection{    grasper.joint.x-grasper.rightFinger.x,
                                                    grasper.joint.y-grasper.rightFinger.y,
                                                    grasper.joint.z-grasper.rightFinger.z};
                                                    
        std::vector<float> rodDirection{    grasper.direction.x,
                                            grasper.direction.y,
                                            grasper.direction.z};
        
        float u=sqrt(   rightFingerDirection[0]*rightFingerDirection[0]
                       +rightFingerDirection[1]*rightFingerDirection[1]
                       +rightFingerDirection[2]*rightFingerDirection[2]);
                    
        for(auto &val:rightFingerDirection) val/=u;

        std::vector<float> rodTranslation={grasper.joint.x,grasper.joint.y,grasper.joint.z};
        std::vector<float> fingerRightObserved={grasper.rightFinger.x,grasper.rightFinger.y,grasper.rightFinger.z};

        getFingerAngle(rightFingerDirection, rodDirection, grasper.angle); 
        getFingerRotation(rodDirection,rodTranslation,fingerRightObserved,-grasper.angle,grasper.rotation);
        // adjust since right is reverse to left
        grasper.rotation=-grasper.rotation;
    }
    else if(pjLeftSize>=2 && pjRightSize>=2){
        std::vector<float> leftFingerDirection{ grasper.joint.x-grasper.leftFinger.x,
                                                grasper.joint.y-grasper.leftFinger.y,
                                                grasper.joint.z-grasper.leftFinger.z};
        
        std::vector<float> rightFingerDirection{    grasper.joint.x-grasper.rightFinger.x,
                                                    grasper.joint.y-grasper.rightFinger.y,
                                                    grasper.joint.z-grasper.rightFinger.z};

        std::vector<float> rodDirection{    grasper.direction.x,
                                            grasper.direction.y,
                                            grasper.direction.z};

        float uLeft=sqrt(    leftFingerDirection[0]*leftFingerDirection[0]
                            +leftFingerDirection[1]*leftFingerDirection[1]
                            +leftFingerDirection[2]*leftFingerDirection[2]);

        float uRight=sqrt(   rightFingerDirection[0]*rightFingerDirection[0]
                            +rightFingerDirection[1]*rightFingerDirection[1]
                            +rightFingerDirection[2]*rightFingerDirection[2]);
        
        for(auto &val:leftFingerDirection) val/=uLeft;
        for(auto &val:rightFingerDirection) val/=uRight;

        float leftRotation=0;
        float rightRotation=0;
        float leftAngle=0;
        float rightAngle=0;

        getFingerAngle(leftFingerDirection, rodDirection, leftAngle); 
        getFingerAngle(rightFingerDirection, rodDirection, rightAngle); 
        grasper.angle=(leftAngle+rightAngle)/2;

        std::vector<float> rodTranslation={grasper.joint.x,grasper.joint.y,grasper.joint.z};
        std::vector<float> fingerLeftObserved={grasper.leftFinger.x,grasper.leftFinger.y,grasper.leftFinger.z};
        std::vector<float> fingerRightObserved={grasper.rightFinger.x,grasper.rightFinger.y,grasper.rightFinger.z};

        getFingerRotation(rodDirection,rodTranslation,fingerLeftObserved,grasper.angle,leftRotation);
        getFingerRotation(rodDirection,rodTranslation,fingerRightObserved,-grasper.angle,rightRotation);

        if(leftRotation>0 && rightRotation<0){
            rightRotation+=M_PI;
        }
        else if(leftRotation<0 && rightRotation>0){
            leftRotation+=M_PI;
        }
        else if(leftRotation>M_PI && rightRotation<M_PI){
            rightRotation+=M_PI;
        }
        else if(leftRotation<M_PI && rightRotation>M_PI){
            leftRotation+=M_PI;
        }

        grasper.rotation=leftRotation;

        
    }
}

// adjust reconstructed grasper tips angle 
float Reconstructor::FingerAngleAdjust(std::vector<float> grasperDirection){
    float d1 = grasperDirection[0], d2 = grasperDirection[1], d3 = grasperDirection[2];

    std::vector<float> azel(2, 0);
    if ( d1 == 0 && d2 == 0 ){
        azel[0] = 0;
        azel[1] = 0;
    } 
    else if ( (d1 < 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 > 0 && d3 < 0) ){

        azel[0] = atan( d2 / d1 ) + 1.5*M_PI;
        azel[1] = 0;
    }
    else{
        azel[0] = atan( d2 / d1 ) + 0.5*M_PI;
        azel[1] = 0;   
    }
    
    return azel[0];
}

// Reference: @cite HartleyZ00 12.2 pag.312
/************************** Triangulating Points**********************************/
template<typename T>
void
Reconstructor::homogeneousToEuclidean(const cv::Mat & _X, cv::Mat & _x)
{
    int d = _X.rows - 1;

    const cv::Mat_<T> & X_rows = _X.rowRange(0,d);
    const cv::Mat_<T> h = _X.row(d);

    const T * h_ptr = h[0], *h_ptr_end = h_ptr + h.cols;
    const T * X_ptr = X_rows[0];
    T * x_ptr = _x.ptr<T>(0);
    for (; h_ptr != h_ptr_end; ++h_ptr, ++X_ptr, ++x_ptr)
    {
        const T * X_col_ptr = X_ptr;
        T * x_col_ptr = x_ptr, *x_col_ptr_end = x_col_ptr + d * _x.step1();
        for (; x_col_ptr != x_col_ptr_end; X_col_ptr+=X_rows.step1(), x_col_ptr+=_x.step1() )
        *x_col_ptr = (*X_col_ptr) / (*h_ptr);
    }
}

void
Reconstructor::homogeneousToEuclidean(cv::InputArray _X, cv::OutputArray _x)
{
    // src
    const cv::Mat X = _X.getMat();

    // dst
    _x.create(X.rows-1, X.cols, X.type());
    cv::Mat x = _x.getMat();

    // type
    if( X.depth() == CV_32F )
    {
        homogeneousToEuclidean<float>(X,x);
    }
    else
    {
        homogeneousToEuclidean<double>(X,x);
    }
}

void Reconstructor::triangulateNViews(const cv::Mat_<double> &x, const std::vector<cv::Matx34d> &Ps, cv::Vec3d &X){
    CV_Assert(x.rows == 2);
    unsigned nviews = x.cols;
    CV_Assert(nviews == Ps.size());

    cv::Mat_<double> design = cv::Mat_<double>::zeros(3*nviews, 4 + nviews);
    for (unsigned i=0; i < nviews; ++i) {
        for(char jj=0; jj<3; ++jj)
            for(char ii=0; ii<4; ++ii)
                design(3*i+jj, ii) = -Ps[i](jj, ii);
        design(3*i + 0, 4 + i) = x(0, i);
        design(3*i + 1, 4 + i) = x(1, i);
        design(3*i + 2, 4 + i) = 1.0;
    }

    cv::Mat X_and_alphas;
    cv::SVD::solveZ(design, X_and_alphas);

    homogeneousToEuclidean(X_and_alphas.rowRange(0, 4), X);
}

void Reconstructor::triangulateDLT( const cv::Vec2d &xl, const cv::Vec2d &xr,
                const cv::Matx34d &Pl, const cv::Matx34d &Pr,
                cv::Vec3d &point3d ){
    cv::Matx44d design;
    for (int i = 0; i < 4; ++i)
    {
        design(0,i) = xl(0) * Pl(2,i) - Pl(0,i);
        design(1,i) = xl(1) * Pl(2,i) - Pl(1,i);
        design(2,i) = xr(0) * Pr(2,i) - Pr(0,i);
        design(3,i) = xr(1) * Pr(2,i) - Pr(1,i);
    }

    cv::Vec4d XHomogeneous;
    cv::SVD::solveZ(design, XHomogeneous);

    homogeneousToEuclidean(XHomogeneous, point3d);
}

void Reconstructor::triangulatePoints(cv::InputArrayOfArrays _points2d, cv::InputArrayOfArrays _projection_matrices, cv::OutputArray _points3d){
    unsigned int nviews = (unsigned) _points2d.total();
    if(nviews<2 || nviews != _projection_matrices.total()) return;

    // inputs
    unsigned int n_points;
    std::vector<cv::Mat_<double> > p2d(nviews);
    std::vector<cv::Matx34d> projection_matrices(nviews);
    {
        std::vector<cv::Mat> points2d_tmp;
        _points2d.getMatVector(points2d_tmp);
        n_points = points2d_tmp[0].cols;

        std::vector<cv::Mat> projection_matrices_tmp;
        _projection_matrices.getMatVector(projection_matrices_tmp);

        // Make sure the dimensions are right
        for(unsigned int i=0; i<nviews; ++i) {
        CV_Assert(points2d_tmp[i].rows == 2 && points2d_tmp[i].cols == n_points);
        if (points2d_tmp[i].type() == CV_64F)
            p2d[i] = points2d_tmp[i];
        else
            points2d_tmp[i].convertTo(p2d[i], CV_64F);

        CV_Assert(projection_matrices_tmp[i].rows == 3 && projection_matrices_tmp[i].cols == 4);
        if (projection_matrices_tmp[i].type() == CV_64F)
            projection_matrices[i] = projection_matrices_tmp[i];
        else
            projection_matrices_tmp[i].convertTo(projection_matrices[i], CV_64F);
        }
    }

        // output
    _points3d.create(3, n_points, CV_64F);
    cv::Mat points3d = _points3d.getMat();

    // Two view
    if( nviews == 2 )
    {
        const cv::Mat_<double> &xl = p2d[0], &xr = p2d[1];
        const cv::Matx34d & Pl = projection_matrices[0];    // left matrix projection
        const cv::Matx34d & Pr = projection_matrices[1];    // right matrix projection

        // triangulate
        for( unsigned i = 0; i < n_points; ++i )
        {
        cv::Vec3d point3d;
        triangulateDLT( cv::Vec2d(xl(0,i), xl(1,i)), cv::Vec2d(xr(0,i), xr(1,i)), Pl, Pr, point3d );
        for(char j=0; j<3; ++j)
            points3d.at<double>(j, i) = point3d[j];
        }
    }
        else if( nviews > 2 )
    {
        // triangulate
        for( unsigned i=0; i < n_points; ++i )
        {
        // build x matrix (one point per view)
        cv::Mat_<double> x( 2, nviews );
        for( unsigned k=0; k < nviews; ++k )
        {
            p2d.at(k).col(i).copyTo( x.col(k) );
        }

        cv::Vec3d point3d;
        triangulateNViews( x, projection_matrices, point3d );
        for(char j=0; j<3; ++j)
            points3d.at<double>(j, i) = point3d[j];
        }
    }   
}

void Reconstructor::normalized_triangulatePoints(std::vector<cv::Mat>& ProjMatrices, std::vector<cv::Mat>& p2d, cv::Mat& point3d, double rows, double cols) {
    // construct normalization matrix
    cv::Mat tran = (cv::Mat_<double>(3,3)<< 2 / cols,   0,      -1,
                                            0       , 2 / rows, -1,
                                            0       ,   0,        1);
        // construct transpoise porjection matrix
    std::vector<cv::Mat> trans_ProjMatrices;
    for (int i = 0; i < ProjMatrices.size(); i++)
        trans_ProjMatrices.push_back(tran * ProjMatrices[i]);

    // construct transpose points
    std::vector<cv::Mat> t_p2d;
    for (int i = 0; i < p2d.size(); i++) {
        cv::Mat p = (cv::Mat_<double>(3,1)<<p2d[i].at<double>(0, 0), p2d[i].at<double>(1, 0), 1);
        cv::Mat tp = tran * p;
        t_p2d.push_back((cv::Mat_<double>(2,1)<<tp.at<double>(0,0), tp.at<double>(1,0)));
    }
    triangulatePoints(t_p2d, trans_ProjMatrices, point3d);
}

