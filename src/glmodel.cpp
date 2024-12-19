// This module defines the opengl 3D model

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include OpenGL
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "glmodel.hpp"
#include "config.hpp"
#include "pose.hpp"
#include "objloader.hpp"

GlModel::GlModel(){}
GlModel::~GlModel(){}

BeanModel::BeanModel(EasyVisConfig* config){
    autoScalor=config->FOREGROUND_SCALOR[0]*1.5;
    generateBall();
}

BeanModel::~BeanModel(){}

// vertices to triangle surfaces
void GrasperModel::generateTriangleList(std::vector<std::vector<std::vector<double>>> modelRaw,
                                        int k,
                                        std::vector<glm::vec3> &vertice,
                                        std::vector<glm::vec2> &uv,
                                        std::string objectType){

  for (int l = 0; l < modelRaw.size() - 1; l++){
        for (int i = 0; i < k; i++){
        //two triangles: 1. (k_top, k_bottom, (k+1)_bottom), 2. (k_top, (k+1)_bottom, (k+1)_top)
        //triangle 1
        glm::vec3 v1, v2, v3;
        v1.x = modelRaw[l+1][i % k][0];
        v1.y = modelRaw[l+1][i % k][1];
        v1.z = modelRaw[l+1][i % k][2];
        v2.x = modelRaw[l][i % k][0];
        v2.y = modelRaw[l][i % k][1];
        v2.z = modelRaw[l][i % k][2];
        v3.x = modelRaw[l][(i + 1) % k][0];
        v3.y = modelRaw[l][(i + 1) % k][1];
        v3.z = modelRaw[l][(i + 1) % k][2];
        vertice.push_back(v1);
        vertice.push_back(v2);
        vertice.push_back(v3);

        //triangle 2
        glm::vec3 v4, v5, v6;
        v4.x = modelRaw[l+1][i % k][0];
        v4.y = modelRaw[l+1][i % k][1];
        v4.z = modelRaw[l+1][i % k][2];
        v5.x = modelRaw[l][(i + 1) % k][0];
        v5.y = modelRaw[l][(i + 1) % k][1];
        v5.z = modelRaw[l][(i + 1) % k][2];
        v6.x = modelRaw[l+1][(i + 1) % k][0];
        v6.y = modelRaw[l+1][(i + 1) % k][1];
        v6.z = modelRaw[l+1][(i + 1) % k][2];
        vertice.push_back(v4);
        vertice.push_back(v5);
        vertice.push_back(v6);

        double offestIdx = (l % 2 == 0) ? 2 : 1;

        //construct corresponding UV coordinates
        glm::vec2 uv1, uv2, uv3;
        
        if(objectType=="finger"){
            //vertical from bottom to up is y, horizontal from left to right is x
            uv1.y = 0.07; uv1.x = 0.156; 
            uv2.y = 0.07; uv2.x = 0.156;
            uv3.y = 0.07; uv3.x = 0.156;
        }
        else if(objectType=="rod"){
            uv1.y = 0.75; uv1.x = 0.156; 
            uv2.y = 0.75; uv2.x = 0.156;
            uv3.y = 0.75; uv3.x = 0.156;
            
        }
        else{

        }

        //first triangle
        uv.push_back(uv1);
        uv.push_back(uv2);
        uv.push_back(uv3);
        //second triangle
        uv.push_back(uv1);
        uv.push_back(uv2);
        uv.push_back(uv3);
        }
    }
}

// generate bean model
void BeanModel::generateBall(){
    double radius=autoScalor*1.3;
    std::vector<std::vector<double>> rawModel;
    std::vector<std::vector<int>> triangleIndices;
    initializeSphere(rawModel, triangleIndices);
    for(int i = 0; i < rawModel.size(); i++){
        rawModel[i][0] = rawModel[i][0] * radius;
        rawModel[i][1] = rawModel[i][1] * radius;
        rawModel[i][2] = rawModel[i][2] * radius;
    }

    for (int i = 0; i < triangleIndices.size() - 1; i++){
        std::vector<int> idx = triangleIndices[i];
        glm::vec3 v1, v2, v3;
        v1.x = rawModel[idx[0]][0];
        v1.y = rawModel[idx[0]][1];
        v1.z = rawModel[idx[0]][2];
        v2.x = rawModel[idx[1]][0];
        v2.y = rawModel[idx[1]][1];
        v2.z = rawModel[idx[1]][2];
        v3.x = rawModel[idx[2]][0];
        v3.y = rawModel[idx[2]][1];
        v3.z = rawModel[idx[2]][2];
        bean.Vertices.push_back(v1);
        bean.Vertices.push_back(v2);
        bean.Vertices.push_back(v3);

        glm::vec2 uv1, uv2, uv3;
        uv1.y = 0.5; uv1.x = 0.156;
        uv2.y = 0.5; uv2.x = 0.156;
        uv3.y = 0.5; uv3.x = 0.156;

        bean.Uvs.push_back(uv1);
        bean.Uvs.push_back(uv2);
        bean.Uvs.push_back(uv3);
    }
}

void BeanModel::normalizeBean(std::vector<double> &v){
    double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] = v[0] / norm;
    v[1] = v[1] / norm;
    v[2] = v[2] / norm;
}

// vertices to surface
void BeanModel::subDivideBean(   std::vector<std::vector<double>> vdata,
                                    int idx[3],
                                    std::vector<std::vector<double>> &sphere_points,
                                    std::vector<std::vector<int>> &tri_indices){
    std::vector<double> v1 = vdata[idx[0]]; 
    std::vector<double> v2 = vdata[idx[1]]; 
    std::vector<double> v3 = vdata[idx[2]]; 
    std::vector<double> v12{v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]};
    std::vector<double> v23{v2[0] + v3[0], v2[1] + v3[1], v2[2] + v3[2]};
    std::vector<double> v31{v3[0] + v1[0], v3[1] + v1[1], v3[2] + v1[2]};
    int end = sphere_points.size() - 1;
    normalizeBean(v12);
    normalizeBean(v23);
    normalizeBean(v31);
    sphere_points.push_back(v12);
    sphere_points.push_back(v23);
    sphere_points.push_back(v31);
    tri_indices.push_back({idx[0], end + 1, end + 3});
    tri_indices.push_back({idx[1], end + 2, end + 1});
    tri_indices.push_back({idx[2], end + 2, end + 3});
    tri_indices.push_back({end + 1, end + 2, end + 3});
}

// generate ball shape
void BeanModel::initializeSphere(std::vector<std::vector<double>> &sphere_points, 
                                    std::vector<std::vector<int>> &tri_indices){  
    double X = 0.5;
    double Y = 0.5;
    double Z = 0.5;
    std::vector<std::vector<double>> vdata = {
        { X, Y, Z}, { -X, Y, Z},{ -X, -Y, Z},{ X, -Y, Z},{ X, Y, -Z},{ -X, Y, -Z},{ -X, -Y, -Z},{ X, -Y, -Z}
    };
    int tindices[20][3] = {
        {0, 1, 2}, {2, 3, 0}, {0 ,1 ,5}, {5, 4, 0},{1,2,5},{5,6,2},{2,3,6},{6,7,3},{4,5,6},{6,7,4},{0,3,4},{4,7,3}
    };

    sphere_points = {
        { X, Y, Z}, { -X, Y, Z},{ -X, -Y, Z},{ X, -Y, Z},{ X, Y, -Z},{ -X, Y, -Z},{ -X, -Y, -Z},{ X, -Y, -Z}
    };

    for(int i = 0; i < 12; i++){
        subDivideBean(vdata, 
                    tindices[i],
                    sphere_points, 
                    tri_indices);
    }
}

// generate grasper finger model
void GrasperModel::generateFingerLeft(){
    double l=autoScalor;
    std::vector<std::vector<std::vector<double>>> rawModel;

    double lengthScaler=8.0;
    std::vector<std::vector<double>> pointsLevel1;
    pointsLevel1.push_back(std::vector<double>{l,-l,-lengthScaler*l});
    pointsLevel1.push_back(std::vector<double>{-l,-l,-lengthScaler*l});
    pointsLevel1.push_back(std::vector<double>{-l,0,-lengthScaler*l});
    pointsLevel1.push_back(std::vector<double>{l,0,-lengthScaler*l});

    std::vector<std::vector<double>> pointsLevel2;
    pointsLevel2.push_back(std::vector<double>{l,-l,-lengthScaler*l+0.25*l});
    pointsLevel2.push_back(std::vector<double>{-l,-l,-lengthScaler*l+0.25*l});
    pointsLevel2.push_back(std::vector<double>{-l,-0.5*l,-lengthScaler*l+0.25*l});
    pointsLevel2.push_back(std::vector<double>{l,-0.5*l,-lengthScaler*l+0.25*l});

    std::vector<std::vector<double>> pointsLevel3;
    pointsLevel3.push_back(std::vector<double>{l,-l,-lengthScaler*l+0.75*l});
    pointsLevel3.push_back(std::vector<double>{-l,-l,-lengthScaler*l+0.75*l});
    pointsLevel3.push_back(std::vector<double>{-l,0,-lengthScaler*l+0.75*l});
    pointsLevel3.push_back(std::vector<double>{l,0,-lengthScaler*l+0.75*l});

    std::vector<std::vector<double>> pointsLevel4;
    pointsLevel4.push_back(std::vector<double>{l,-l,-lengthScaler*l+1.25*l});
    pointsLevel4.push_back(std::vector<double>{-l,-l,-lengthScaler*l+1.25*l});
    pointsLevel4.push_back(std::vector<double>{-l,-0.5*l,-lengthScaler*l+1.25*l});
    pointsLevel4.push_back(std::vector<double>{l,-0.5*l,-lengthScaler*l+1.25*l});

    std::vector<std::vector<double>> pointsLevel5;
    pointsLevel5.push_back(std::vector<double>{l,-l,-lengthScaler*l+1.75*l});
    pointsLevel5.push_back(std::vector<double>{-l,-l,-lengthScaler*l+1.75*l});
    pointsLevel5.push_back(std::vector<double>{-l,0,-lengthScaler*l+1.75*l});
    pointsLevel5.push_back(std::vector<double>{l,0,-lengthScaler*l+1.75*l});

    std::vector<std::vector<double>> pointsLevel6;
    pointsLevel6.push_back(std::vector<double>{l,-l,-lengthScaler*l+2.25*l});
    pointsLevel6.push_back(std::vector<double>{-l,-l,-lengthScaler*l+2.25*l});
    pointsLevel6.push_back(std::vector<double>{-l,-0.5*l,-lengthScaler*l+2.25*l});
    pointsLevel6.push_back(std::vector<double>{l,-0.5*l,-lengthScaler*l+2.25*l});

    std::vector<std::vector<double>> pointsLevel7;
    pointsLevel7.push_back(std::vector<double>{l,-l,-lengthScaler*l+2.75*l});
    pointsLevel7.push_back(std::vector<double>{-l,-l,-lengthScaler*l+2.75*l});
    pointsLevel7.push_back(std::vector<double>{-l,0,-lengthScaler*l+2.75*l});
    pointsLevel7.push_back(std::vector<double>{l,0,-lengthScaler*l+2.75*l});

    std::vector<std::vector<double>> pointsLevel8;
    pointsLevel8.push_back(std::vector<double>{l,-l,-lengthScaler*l+3.25*l});
    pointsLevel8.push_back(std::vector<double>{-l,-l,-lengthScaler*l+3.25*l});
    pointsLevel8.push_back(std::vector<double>{-l,-0.5*l,-lengthScaler*l+3.25*l});
    pointsLevel8.push_back(std::vector<double>{l,-0.5*l,-lengthScaler*l+3.25*l});

    std::vector<std::vector<double>> pointsLevel9;
    pointsLevel9.push_back(std::vector<double>{l,-l,0});
    pointsLevel9.push_back(std::vector<double>{-l,-l,0});
    pointsLevel9.push_back(std::vector<double>{-l,-0.5*l,0});
    pointsLevel9.push_back(std::vector<double>{l,-0.5*l,0});

    rawModel.push_back(pointsLevel1);
    rawModel.push_back(pointsLevel2);
    rawModel.push_back(pointsLevel3);
    rawModel.push_back(pointsLevel4);
    rawModel.push_back(pointsLevel5);
    rawModel.push_back(pointsLevel6);
    rawModel.push_back(pointsLevel7);
    rawModel.push_back(pointsLevel8);
    rawModel.push_back(pointsLevel9);

    generateTriangleList(rawModel,4,fingerLeft.Vertices,fingerLeft.Uvs,"finger");
}

// generate grasper finger model
void GrasperModel::generateFingerRight(){
    double l=autoScalor;
    std::vector<std::vector<std::vector<double>>> rawModel;

    double lengthScaler=8.0;

    std::vector<std::vector<double>> pointsLevel1;
    pointsLevel1.push_back(std::vector<double>{l,l,-lengthScaler*l});
    pointsLevel1.push_back(std::vector<double>{-l,l,-lengthScaler*l});
    pointsLevel1.push_back(std::vector<double>{-l,0,-lengthScaler*l});
    pointsLevel1.push_back(std::vector<double>{l,0,-lengthScaler*l});

    std::vector<std::vector<double>> pointsLevel2;
    pointsLevel2.push_back(std::vector<double>{l,l,-lengthScaler*l+0.25*l});
    pointsLevel2.push_back(std::vector<double>{-l,l,-lengthScaler*l+0.25*l});
    pointsLevel2.push_back(std::vector<double>{-l,0.5*l,-lengthScaler*l+0.25*l});
    pointsLevel2.push_back(std::vector<double>{l,0.5*l,-lengthScaler*l+0.25*l});

    std::vector<std::vector<double>> pointsLevel3;
    pointsLevel3.push_back(std::vector<double>{l,l,-lengthScaler*l+0.75*l});
    pointsLevel3.push_back(std::vector<double>{-l,l,-lengthScaler*l+0.75*l});
    pointsLevel3.push_back(std::vector<double>{-l,0,-lengthScaler*l+0.75*l});
    pointsLevel3.push_back(std::vector<double>{l,0,-lengthScaler*l+0.75*l});

    std::vector<std::vector<double>> pointsLevel4;
    pointsLevel4.push_back(std::vector<double>{l,l,-lengthScaler*l+1.25*l});
    pointsLevel4.push_back(std::vector<double>{-l,l,-lengthScaler*l+1.25*l});
    pointsLevel4.push_back(std::vector<double>{-l,0.5*l,-lengthScaler*l+1.25*l});
    pointsLevel4.push_back(std::vector<double>{l,0.5*l,-lengthScaler*l+1.25*l});

    std::vector<std::vector<double>> pointsLevel5;
    pointsLevel5.push_back(std::vector<double>{l,l,-lengthScaler*l+1.75*l});
    pointsLevel5.push_back(std::vector<double>{-l,l,-lengthScaler*l+1.75*l});
    pointsLevel5.push_back(std::vector<double>{-l,0,-lengthScaler*l+1.75*l});
    pointsLevel5.push_back(std::vector<double>{l,0,-lengthScaler*l+1.75*l});

    std::vector<std::vector<double>> pointsLevel6;
    pointsLevel6.push_back(std::vector<double>{l,l,-lengthScaler*l+2.25*l});
    pointsLevel6.push_back(std::vector<double>{-l,l,-lengthScaler*l+2.25*l});
    pointsLevel6.push_back(std::vector<double>{-l,0.5*l,-lengthScaler*l+2.25*l});
    pointsLevel6.push_back(std::vector<double>{l,0.5*l,-lengthScaler*l+2.25*l});

    std::vector<std::vector<double>> pointsLevel7;
    pointsLevel7.push_back(std::vector<double>{l,l,-lengthScaler*l+2.75*l});
    pointsLevel7.push_back(std::vector<double>{-l,l,-lengthScaler*l+2.75*l});
    pointsLevel7.push_back(std::vector<double>{-l,0,-lengthScaler*l+2.75*l});
    pointsLevel7.push_back(std::vector<double>{l,0,-lengthScaler*l+2.75*l});

    std::vector<std::vector<double>> pointsLevel8;
    pointsLevel8.push_back(std::vector<double>{l,l,-lengthScaler*l+3.25*l});
    pointsLevel8.push_back(std::vector<double>{-l,l,-lengthScaler*l+3.25*l});
    pointsLevel8.push_back(std::vector<double>{-l,0.5*l,-lengthScaler*l+3.25*l});
    pointsLevel8.push_back(std::vector<double>{l,0.5*l,-lengthScaler*l+3.25*l});

    std::vector<std::vector<double>> pointsLevel9;
    pointsLevel9.push_back(std::vector<double>{l,l,0});
    pointsLevel9.push_back(std::vector<double>{-l,l,0});
    pointsLevel9.push_back(std::vector<double>{-l,0.5*l,0});
    pointsLevel9.push_back(std::vector<double>{l,0.5*l,0});

    rawModel.push_back(pointsLevel1);
    rawModel.push_back(pointsLevel2);
    rawModel.push_back(pointsLevel3);
    rawModel.push_back(pointsLevel4);
    rawModel.push_back(pointsLevel5);
    rawModel.push_back(pointsLevel6);
    rawModel.push_back(pointsLevel7);
    rawModel.push_back(pointsLevel8);
    rawModel.push_back(pointsLevel9);

    generateTriangleList(rawModel,4,fingerRight.Vertices,fingerRight.Uvs,"finger");
}

// generate grasper rod
void GrasperModel::generateRodSimple(){
    std::vector<std::vector<std::vector<double>>> rawModel;

	double r = rodRadius;
    int k=rodResolution;
    double h=rodRadius*8;
    int n=rodLength;

	for (int l = 0; l < n; l++){
		std::vector<std::vector<double>> cur_layer;
		for (int i = 0; i < k; i++){
			double theta = 2 * M_PI * i / k;
			cur_layer.push_back(std::vector<double>{r * sin(theta), r * cos(theta), l*h});
		}
		rawModel.push_back(cur_layer);
	}

    generateTriangleList(rawModel,k,rod.Vertices,rod.Uvs,"rod");
}

// generate grasper rod
void GrasperModel::generateRodComplex(){
    std::vector<std::vector<std::vector<double>>> rawModel;

	double r = rodRadius*1.2;
    int k=rodResolution;
    double h=rodRadius;
    int n=rodLength;

    //define points
    std::vector<std::vector<double>> cur_layer1a;
    for (int i = 0; i < k; i++){
        double theta = 2 * M_PI * i / k;
        cur_layer1a.push_back(std::vector<double>{r *0.9 * sin(theta), r *0.9 * cos(theta), 0});
    }
    rawModel.push_back(cur_layer1a);

    std::vector<std::vector<double>> cur_layer1b;
    for (int i = 0; i < k; i++){
        double theta = 2 * M_PI * i / k;
        cur_layer1b.push_back(std::vector<double>{r *0.9 * sin(theta), r *0.9 * cos(theta), h*3});
    }
    rawModel.push_back(cur_layer1b);

    std::vector<std::vector<double>> cur_layer2;
    for (int i = 0; i < k; i++){
        double theta = 2 * M_PI * i / k;
        cur_layer2.push_back(std::vector<double>{r * sin(theta), r * cos(theta), h*3});
    }
    rawModel.push_back(cur_layer2);

    std::vector<std::vector<double>> cur_layer3;
    for (int i = 0; i < k; i++){
        double theta = 2 * M_PI * i / k;
        cur_layer3.push_back(std::vector<double>{r * sin(theta), r * cos(theta), h*7});
    }
    rawModel.push_back(cur_layer3);

    std::vector<std::vector<double>> cur_layer4;
    for (int i = 0; i < k; i++){
        double theta = 2 * M_PI * i / k;
        cur_layer4.push_back(std::vector<double>{r * sin(theta), r * cos(theta), h*9});
    }
    rawModel.push_back(cur_layer4);

    std::vector<std::vector<double>> cur_layer5;
    for (int i = 0; i < k; i++){
        double theta = 2 * M_PI * i / k;
        cur_layer5.push_back(std::vector<double>{r * sin(theta), r * cos(theta), h*8*n});
    }
    rawModel.push_back(cur_layer5);

    //generate triangle list
    for (int l = 0; l < rawModel.size() - 1; l++){
        for (int i = 0; i < k; i++){
        //two triangles: 1. (k_top, k_bottom, (k+1)_bottom), 2. (k_top, (k+1)_bottom, (k+1)_top)
        //triangle 1
        glm::vec3 v1, v2, v3;
        v1.x = rawModel[l+1][i % k][0];
        v1.y = rawModel[l+1][i % k][1];
        v1.z = rawModel[l+1][i % k][2];
        v2.x = rawModel[l][i % k][0];
        v2.y = rawModel[l][i % k][1];
        v2.z = rawModel[l][i % k][2];
        v3.x = rawModel[l][(i + 1) % k][0];
        v3.y = rawModel[l][(i + 1) % k][1];
        v3.z = rawModel[l][(i + 1) % k][2];
        rod.Vertices.push_back(v1);
        rod.Vertices.push_back(v2);
        rod.Vertices.push_back(v3);

        //triangle 2
        glm::vec3 v4, v5, v6;
        v4.x = rawModel[l+1][i % k][0];
        v4.y = rawModel[l+1][i % k][1];
        v4.z = rawModel[l+1][i % k][2];
        v5.x = rawModel[l][(i + 1) % k][0];
        v5.y = rawModel[l][(i + 1) % k][1];
        v5.z = rawModel[l][(i + 1) % k][2];
        v6.x = rawModel[l+1][(i + 1) % k][0];
        v6.y = rawModel[l+1][(i + 1) % k][1];
        v6.z = rawModel[l+1][(i + 1) % k][2];
        rod.Vertices.push_back(v4);
        rod.Vertices.push_back(v5);
        rod.Vertices.push_back(v6);

        double offestIdx = (l % 2 == 0) ? 2 : 1;

        //construct corresponding UV coordinates
        glm::vec2 uv1, uv2, uv3;
        
        if(l==0){
            uv1.y = 0.07; uv1.x = 0.156; 
            uv2.y = 0.07; uv2.x = 0.156;
            uv3.y = 0.07; uv3.x = 0.156;
        }
        else if(l==3){
            uv1.y = 0.3; uv1.x = 0.156; 
            uv2.y = 0.3; uv2.x = 0.156;
            uv3.y = 0.3; uv3.x = 0.156;
        }
        else{
            uv1.y = 0.75; uv1.x = 0.156; 
            uv2.y = 0.75; uv2.x = 0.156;
            uv3.y = 0.75; uv3.x = 0.156;
        }
        
            

        //first triangle
        rod.Uvs.push_back(uv1);
        rod.Uvs.push_back(uv2);
        rod.Uvs.push_back(uv3);
        //second triangle
        rod.Uvs.push_back(uv1);
        rod.Uvs.push_back(uv2);
        rod.Uvs.push_back(uv3);
        }
    }

}

// generate grasper model
GrasperModel::GrasperModel(EasyVisConfig* config){
    autoScalor=config->FOREGROUND_SCALOR[0];
    rodRadius=config->FOREGROUND_SCALOR[0];
    rodResolution=int(config->FOREGROUND_SCALOR[1]);
    rodLength=int(config->FOREGROUND_SCALOR[2]);

    generateFingerLeft();
    generateFingerRight();
    //generateRodSimple();
    generateRodComplex();
}

GrasperModel::~GrasperModel(){}

// load static background model
BackgroundModel::BackgroundModel(EasyVisConfig *config, double UVScale, double UVOffset){
    // Read our .obj file
    loadOBJ(config->BACKGROUND_MODEL_PATH.c_str(), Vertices, Uvs, Normals, allVertices);

    //adjust UVs after creating new textures
    for (auto& Uv : Uvs){
        Uv[1] = Uv[1] * UVScale + UVOffset;
    }
}
groundModel(){}
