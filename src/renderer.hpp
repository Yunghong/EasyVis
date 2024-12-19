// 3D rendering module

#ifndef RENDERER_H
#define RENDERER_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>

#include "renderer.hpp"
#include "shader.hpp"
#include "glmodel.hpp"
#include "texture.hpp"
#include "pose.hpp"
#include "glmodel.hpp"
#include "container.hpp"
#include "kalmanfilter.hpp"

class Renderer
{
private:
    // Render params
    GLuint VertexArrayID, programID, MatrixID;
    GLuint uvbuffer;
    GLuint vertexbuffer;
    GLuint Texture;
    GLuint TextureID;
    glm::mat4 centering;
    glm::mat4 offcenter;
    //glm::mat4 Projection;
    glm::mat4 Model;
    glm::mat4 t;
    glm::mat3 rx;
    glm::mat3 ry;
    glm::mat3 rz;
    glm::mat4 View; //construct identity matrix
    //glm::mat4 MVP;
    cv::Mat Rs_rec,T1;

    // Control params
    double move_increamental;
	  double rotation_increamental;
	  double eye_dist;
	  bool stereo;
    bool useForeground;
    float x;
	  float y;
	  float z;
	  float yaw; //in radians
	  float pitch;
	  float roll;

    int viewNumber;

    int setRefView;

    // Save render view
    bool save;
    int saveCounter;
    std::string savePath;

    // Params for adjusting bg uv
    double UVScale;
    double UVOffset;

    // Background point size
    int backgroundSize;
    cv::Point3f bgCenter;

    // preload camera pose
    CameraPose *cameraPose;

    glm::mat4 Projection;

public:
    
    GLFWwindow* window;
    
    BackgroundModel *backgroundModel;
    GrasperModel *grasper;
    BeanModel *bean;

    Renderer(EasyVisConfig *config, CameraPose *pose);

    void Render(std::vector<glm::vec3> &Vertices, std::vector<glm::vec2> &Uvs);
    void InitializeViewParams(CameraPose *pose);
    void generateTextureMap(EasyVisConfig *config);
    void RenderControl();
    void RenderUpdateParam();
    void setupOpenGL();
    void flash();
    void reset();
    void clean();

    void notice(
        std::vector<cv::Mat> imgs, 
        std::vector<std::vector<grasper2d>> graspers,
        std::vector<std::vector<bean2d>> beans);

    glm::mat4 createProjectionFromIntrinsics(
    float fx, float fy,  // focal lengths
    float cx, float cy,  // principal point
    float width, float height,  // image dimensions
    float nearZ, float farZ);


    void run(
        std::unordered_set<int>& bean_blk_Ids,
        std::unordered_set<int>& bean_dkbl_Ids,
        std::unordered_set<int>& bean_gray_Ids,
        std::unordered_set<int>& bean_gren_Ids,
        std::unordered_set<int>& bean_ltbl_Ids,
        std::unordered_set<int>& bean_orng_Ids,
        std::unordered_set<int>& bean_prpl_Ids,
        std::unordered_set<int>& bean_red_Ids,
        std::unordered_set<int>& grasper_B_Ids,
        std::unordered_set<int>& grasper_W_Ids,
        std::unordered_map<int, bean3dKF> &beans_blk_KF3d,
        std::unordered_map<int, bean3dKF> &beans_dkbl_KF3d,
        std::unordered_map<int, bean3dKF> &beans_gray_KF3d,
        std::unordered_map<int, bean3dKF> &beans_gren_KF3d,
        std::unordered_map<int, bean3dKF> &beans_ltbl_KF3d,
        std::unordered_map<int, bean3dKF> &beans_orng_KF3d,
        std::unordered_map<int, bean3dKF> &beans_prpl_KF3d,
        std::unordered_map<int, bean3dKF> &beans_red_KF3d,
        std::unordered_map<int, grasper3dKF> &graspers_B_KF3d,
        std::unordered_map<int, grasper3dKF> &graspers_W_KF3d,
        std::vector<cv::Mat> imgs);

    void runNoTrack(
        std::vector<grasper3d> graspersReconstructed,
        std::vector<bean3d> beansReconstructed,
        std::vector<cv::Mat> imgs);

    void saveNovelView(
        std::vector<cv::Mat> groundTruths,
        std::vector<glm::vec3> &RenderVertices,
        std::vector<glm::vec2> &RenderUvs);

    // finger transfer, finger transfer is different
    std::vector<glm::vec3> transferModelFinger(   
        std::vector<glm::vec3> finger, 
        double rotation, double angle);

    // general transfer
    std::vector<glm::vec3> transferModel(
        std::vector<glm::vec3> model, 
        std::vector<float> direction, 
        std::vector<float> translation,
        float offset);

    glm::vec3 matrixMul(
        glm::vec3 points,
        std::vector<std::vector<double>> rot);

    void getBgCenter();
    
    ~Renderer();
};

#endif
