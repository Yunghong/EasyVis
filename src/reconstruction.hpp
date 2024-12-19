#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <unordered_map>
#include <unordered_set>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "container.hpp"
#include "kalmanfilter.hpp"
#include "config.hpp"
#include "pose.hpp"
#include "BYTETracker.h"

class Reconstructor
{
private:

    int viewNumber; 

    // manage the id number of grasper and bean
    int grasperIdStart;
    int beanIdStart;

    CameraPose *pose;

    // pair first, then reconstruction
    std::vector<std::unordered_map<int, bean2d>> beans_blk_Paired;
    std::vector<std::unordered_map<int, bean2d>> beans_dkbl_Paired;
    std::vector<std::unordered_map<int, bean2d>> beans_gray_Paired;
    std::vector<std::unordered_map<int, bean2d>> beans_gren_Paired;
    std::vector<std::unordered_map<int, bean2d>> beans_ltbl_Paired;
    std::vector<std::unordered_map<int, bean2d>> beans_orng_Paired;
    std::vector<std::unordered_map<int, bean2d>> beans_prpl_Paired;
    std::vector<std::unordered_map<int, bean2d>> beans_red_Paired;

    std::vector<std::unordered_map<int, grasper2d>> graspers_B_Paired;
    std::vector<std::unordered_map<int, grasper2d>> graspers_W_Paired;
    
    std::vector<std::unordered_map<int, bean2dKF>> beans_blk_KF;
    std::vector<std::unordered_map<int, bean2dKF>> beans_dkbl_KF;
    std::vector<std::unordered_map<int, bean2dKF>> beans_gray_KF;
    std::vector<std::unordered_map<int, bean2dKF>> beans_gren_KF;
    std::vector<std::unordered_map<int, bean2dKF>> beans_ltbl_KF;
    std::vector<std::unordered_map<int, bean2dKF>> beans_orng_KF;
    std::vector<std::unordered_map<int, bean2dKF>> beans_prpl_KF;
    std::vector<std::unordered_map<int, bean2dKF>> beans_red_KF;

    std::vector<std::unordered_map<int, grasper2dKF>> graspers_B_KF;
    std::vector<std::unordered_map<int, grasper2dKF>> graspers_W_KF;
    
public:
    // this handle the object numbers in 3D(in real)
    std::unordered_set<int> bean_blk_Ids;
    std::unordered_set<int> bean_dkbl_Ids;
    std::unordered_set<int> bean_gray_Ids;
    std::unordered_set<int> bean_gren_Ids;
    std::unordered_set<int> bean_ltbl_Ids;
    std::unordered_set<int> bean_orng_Ids;
    std::unordered_set<int> bean_prpl_Ids;
    std::unordered_set<int> bean_red_Ids;

    std::unordered_set<int> grasper_B_Ids;
    std::unordered_set<int> grasper_W_Ids;
    

    // 3D container
    std::unordered_map<int, bean3d> beans_blk_3d;
    std::unordered_map<int, bean3d> beans_dkbl_3d;
    std::unordered_map<int, bean3d> beans_gray_3d;
    std::unordered_map<int, bean3d> beans_gren_3d;
    std::unordered_map<int, bean3d> beans_ltbl_3d;
    std::unordered_map<int, bean3d> beans_orng_3d;
    std::unordered_map<int, bean3d> beans_prpl_3d;
    std::unordered_map<int, bean3d> beans_red_3d;

    std::unordered_map<int, grasper3d> graspers_B_3d;
    std::unordered_map<int, grasper3d> graspers_W_3d;
    
    std::unordered_map<int, bean3dKF> beans_blk_KF3d;
    std::unordered_map<int, bean3dKF> beans_dkbl_KF3d;
    std::unordered_map<int, bean3dKF> beans_gray_KF3d;
    std::unordered_map<int, bean3dKF> beans_gren_KF3d;
    std::unordered_map<int, bean3dKF> beans_ltbl_KF3d;
    std::unordered_map<int, bean3dKF> beans_orng_KF3d;
    std::unordered_map<int, bean3dKF> beans_prpl_KF3d;
    std::unordered_map<int, bean3dKF> beans_red_KF3d;

    std::unordered_map<int, grasper3dKF> graspers_B_KF3d;
    std::unordered_map<int, grasper3dKF> graspers_W_KF3d;
    

    std::vector<bean3d> beansReconstructed;
    std::vector<grasper3d> graspersReconstructed;

    Reconstructor(EasyVisConfig *easyVisConfig, CameraPose *cameraPose);

    int getDistance(cv::Point2i X0, cv::Point2i X1);

    double calculateDistance(double x1, double y1, double x2, double y2);

    void prepareSingleObject(
        std::vector<std::vector<bean2d>> Bean_Black,
        std::vector<std::vector<bean2d>> Bean_DarkBlue,
        std::vector<std::vector<bean2d>> Bean_Gray,
        std::vector<std::vector<bean2d>> Bean_Green,
        std::vector<std::vector<bean2d>> Bean_LightBlue,
        std::vector<std::vector<bean2d>> Bean_Orng,
        std::vector<std::vector<bean2d>> Bean_Prpl,
        std::vector<std::vector<bean2d>> Bean_Red,
        std::vector<std::vector<grasper2d>> Grasper_B,
        std::vector<std::vector<grasper2d>> Grasper_W);

    void prepareSingleObject_bean(
        std::vector<std::vector<bean2d>> &beans,
        std::vector<std::unordered_map<int, bean2d>>& beansPaired,
        std::vector<std::unordered_map<int, bean2dKF>>& beansKF,
        std::unordered_set<int>& beanIds);

    void prepareSingleObject_grasper(
        std::vector<std::vector<grasper2d>> &graspers,
        std::vector<std::unordered_map<int, grasper2d>>& graspersPaired,
        std::vector<std::unordered_map<int, grasper2dKF>>& graspersKF,
        std::unordered_set<int>& grasperIds);

    void reconstructSingleObject();

    void reconstructSingleObject_bean(
        std::unordered_set<int>& beanIds,
        std::vector<std::unordered_map<int, bean2d>>& beansPaired,
        std::vector<std::unordered_map<int, bean2dKF>>& beansKF,
        std::unordered_map<int, bean3d>& beans3d,
        std::unordered_map<int, bean3dKF>& beansKF3d);

    void reconstructSingleObject_grasper(
        std::unordered_set<int>& grasperIds,
        std::vector<std::unordered_map<int, grasper2d>>& graspersPaired,
        std::vector<std::unordered_map<int, grasper2dKF>>& graspersKF,
        std::unordered_map<int, grasper3d>& graspers3d,
        std::unordered_map<int, grasper3dKF>& graspersKF3d);


    void run(
        std::vector<std::vector<bean2d>> Bean_Black,
        std::vector<std::vector<bean2d>> Bean_DarkBlue,
        std::vector<std::vector<bean2d>> Bean_Gray,
        std::vector<std::vector<bean2d>> Bean_Green,
        std::vector<std::vector<bean2d>> Bean_LightBlue,
        std::vector<std::vector<bean2d>> Bean_Orng,
        std::vector<std::vector<bean2d>> Bean_Prpl,
        std::vector<std::vector<bean2d>> Bean_Red,
        std::vector<std::vector<grasper2d>> Grasper_B,
        std::vector<std::vector<grasper2d>> Grasper_W);

    template<typename T>
    int getObjectNumber(std::vector<std::vector<T>> objects);

    void reconstructFingerPose(size_t pjLeftSize, size_t pjRightSize, grasper3d &grasper);
    void getFingerRotation(std::vector<float> fingerDirection, float &rotation);
    void getFingerRotation(
        std::vector<float> rodDirection, 
        std::vector<float> rodTranslation, 
        std::vector<float> fingerObserved, 
        float angle, float &rotation);

    std::vector<std::vector<float>> getInverse(const std::vector<std::vector<float>> vect);
    std::vector<std::vector<float>> getCofactor(const std::vector<std::vector<float>> vect);
    float getDeterminant(const std::vector<std::vector<float>> vect);
    std::vector<std::vector<float>> getTranspose(const std::vector<std::vector<float>> matrix1);

    void getFingerAngle(std::vector<float> d1, std::vector<float> d2, float &angle);
    float FingerAngleAdjust(std::vector<float> grasperDirection);
    void summary();

    template<typename T>
    std::vector<int> getValidViewNumber(std::vector<std::vector<T>> objectVector);

    void normalized_triangulatePoints(
        std::vector<cv::Mat>& ProjMatrices, 
        std::vector<cv::Mat>& p2d, 
        cv::Mat& point3d, 
        double rows, 
        double cols);

    void triangulatePoints(
        cv::InputArrayOfArrays _points2d, 
        cv::InputArrayOfArrays _projection_matrices, 
        cv::OutputArray _points3d);

    void triangulateDLT( 
        const cv::Vec2d &xl, 
        const cv::Vec2d &xr,
        const cv::Matx34d &Pl, 
        const cv::Matx34d &Pr,
        cv::Vec3d &point3d );

    void triangulateNViews(
        const cv::Mat_<double> &x, 
        const std::vector<cv::Matx34d> &Ps,
        cv::Vec3d &X);

    void homogeneousToEuclidean(
        cv::InputArray _X, 
        cv::OutputArray _x);

    template<typename T>
    void homogeneousToEuclidean(
        const cv::Mat & _X, 
        cv::Mat & _x);
};

#endif
