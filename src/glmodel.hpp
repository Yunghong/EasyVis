// this module generates opengl models

#ifndef GLMODEL_H
#define GLMODEL_H

// Include OpenGL
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <string>

#include "config.hpp"
#include "pose.hpp"

class GlModel
{
private:

public:
    std::vector<glm::vec3> Vertices;
    std::vector<glm::vec2> Uvs;
    std::vector<glm::vec3> Normals;

    GlModel();
    ~GlModel();
};

class BeanModel
{
private:
    double autoScalor;

public:
    GlModel bean;

    BeanModel(EasyVisConfig* config);

    void generateBall();

    void initializeSphere(  
        std::vector<std::vector<double>> &sphere_points, 
        std::vector<std::vector<int>> &tri_indices);

    void subDivideBean( 
        std::vector<std::vector<double>> vdata,
        int idx[3],
        std::vector<std::vector<double>> &sphere_points,
        std::vector<std::vector<int>> &tri_indices);

    void normalizeBean(std::vector<double> &v);

    ~BeanModel();
};

class GrasperModel
{
private:
    double autoScalor;
    double rodRadius;
    int rodResolution;
    int rodLength;

public:
    GlModel rod, fingerLeft, fingerRight;

    GrasperModel(EasyVisConfig* config);
    void generateFingerLeft();
    void generateFingerRight();
    void generateRodSimple();
    void generateRodComplex();

    void generateTriangleList(  
        std::vector<std::vector<std::vector<double>>> modelRaw,
        int k,
        std::vector<glm::vec3> &vertice,
        std::vector<glm::vec2> &uv,
        std::string objectType);

    ~GrasperModel();
};

class BackgroundModel: public GlModel
{
private:
    // I don't know what's this
    std::vector<std::vector<double>> allVertices;

public:
    // UVSCale and UVOffset use to change the texture map coordinate due to extend the original texture map
    BackgroundModel(EasyVisConfig* config, double UVScale, double UVOffset);
    ~BackgroundModel();
};

#endif
