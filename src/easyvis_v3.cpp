#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include OpenGL
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <unordered_map>
#include <unordered_set>
#include <vector>

//OpenMP
#include <omp.h>
#include <chrono>
#include <thread>
#include <iostream>

#include "camera.hpp"
#include "config.hpp"
#include "pose.hpp"
#include "glmodel.hpp"
#include "renderer.hpp"
#include "detection.hpp"
#include "kalmanfilter.hpp"
#include "reconstruction.hpp"
#include "validation.hpp"
#include "epipolarTracking.hpp"

// #include "old_shit_mountain.cpp"


// Runs object detection on an input image then saves the annotated image to disk.
int main(int argc, char *argv[]) {
    //setting threads
    omp_set_dynamic(0);
    omp_set_num_threads(128);

    // Load configurations
    YoloV8Config yoloConfig;
    EasyVisConfig *easyVisConfig = new EasyVisConfig();

    int timeTotal=0;
    int timeRd=0;
    int timeDet=0;
    int timeDrec=0;
    int timeRend=0;
    int ctimeTotal=0;
    int ctimeRd=0;
    int ctimeDet=0;
    int ctimeDrec=0;
    int ctimeRend=0;
    
    // Camera initialize
    Cameras *cameras= new Cameras(easyVisConfig->CAM_PORTS,easyVisConfig);

    // Camera Pose initialize
    CameraPose *pose = new CameraPose(easyVisConfig);

    // Render engine, the models are initialized in it
    Renderer *renderer= new Renderer(easyVisConfig, pose);

    // initialize detector
    Detection *detector=new Detection(easyVisConfig->MODEL_PATH, yoloConfig, cameras);

    // initialize tracker
    EpipolarTrackor *trackor=new EpipolarTrackor(cameras);

    // initialize reconstructor
    Reconstructor *reconstructor= new Reconstructor(easyVisConfig, pose);

    // initialize validator
    Validation *validator= new Validation(pose,easyVisConfig->VIEW_NUMBER);

    // ini 3D trackor
    BYTETrackerBean3D trackerBean3D(30, 30);

    while(glfwGetKey(renderer->window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(renderer->window) == 0){
        // cam read
        auto start = std::chrono::high_resolution_clock::now();
        cameras->captures();
        
        // 2D detection
        auto timeReadCam = std::chrono::high_resolution_clock::now();
        detector->detect(cameras->imgs);
        
        // 3D reconstruction
        auto timeDetect = std::chrono::high_resolution_clock::now();
        reconstructor->run(
            detector->Bean_Black,
            detector-> Bean_DarkBlue,
            detector->Bean_Gray,
            detector->Bean_Green,
            detector->Bean_LightBlue,
            detector->Bean_Orng,
            detector->Bean_Prpl,
            detector->Bean_Red,
            detector->Grasper_B,
            detector->Grasper_W);

        // 3D rendering
        auto timeRec = std::chrono::high_resolution_clock::now();
        renderer->run(
            reconstructor->bean_blk_Ids,
            reconstructor->bean_dkbl_Ids,
            reconstructor->bean_gray_Ids,
            reconstructor->bean_gren_Ids,
            reconstructor->bean_ltbl_Ids,
            reconstructor->bean_orng_Ids,
            reconstructor->bean_prpl_Ids,
            reconstructor->bean_red_Ids,
            reconstructor->grasper_B_Ids,
            reconstructor->grasper_W_Ids,
            reconstructor->beans_blk_KF3d,
            reconstructor->beans_dkbl_KF3d,
            reconstructor->beans_gray_KF3d,
            reconstructor->beans_gren_KF3d,
            reconstructor->beans_ltbl_KF3d,
            reconstructor->beans_orng_KF3d,
            reconstructor->beans_prpl_KF3d,
            reconstructor->beans_red_KF3d,
            reconstructor->graspers_B_KF3d,
            reconstructor->graspers_W_KF3d,
            cameras->imgs);

        auto timeRender = std::chrono::high_resolution_clock::now(); 
        // summary scores for exp
        validator->BPEAppend(reconstructor->graspers_W_KF3d, reconstructor->beans_blk_KF3d,detector->Grasper_W,detector->Bean_Black);

        auto durationtotal = std::chrono::duration_cast<std::chrono::microseconds>(timeRender - start);
        auto durationReadCam = std::chrono::duration_cast<std::chrono::microseconds>(timeReadCam - start);
        auto durationdetect = std::chrono::duration_cast<std::chrono::microseconds>(timeDetect-timeReadCam);
        auto durationRec = std::chrono::duration_cast<std::chrono::microseconds>(timeRec - timeDetect);
        auto durationRender = std::chrono::duration_cast<std::chrono::microseconds>(timeRender - timeRec);

        timeTotal+=durationtotal.count();
        timeRd+=durationReadCam.count();
        timeDet+=durationdetect.count();
        timeDrec+=durationRec.count();
        timeRend+=durationRender.count();

        ctimeTotal++;
        ctimeRd++;
        ctimeDet++;
        ctimeDrec++;
        ctimeRend++;

      
    }

    validator->BPESummary();
    std::cout<<timeTotal/ctimeTotal <<std::endl;
    std::cout<<timeRd/ctimeRd <<std::endl;
    std::cout<<timeDet/ctimeDet <<std::endl;
    std::cout<<timeDrec/ctimeDrec <<std::endl;
    std::cout<<timeRend/ctimeRend <<std::endl;

    renderer->clean();
    delete easyVisConfig;
    delete renderer;
    delete cameras;
    delete pose;
    delete detector;
    delete reconstructor;
    delete validator;
    delete trackor;

    return 0;
}


