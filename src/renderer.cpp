// this module handle 3D rendering

// Include OpenGL

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

//OpenMP
#include <omp.h>
#include <chrono>
#include <thread>

#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "FreeImage.h"
#include <math.h> 

#include "renderer.hpp"
#include "shader.hpp"
#include "glmodel.hpp"
#include "texture.hpp"
#include "pose.hpp"
#include "glmodel.hpp"
#include "container.hpp"
#include "kalmanfilter.hpp"

//image saver
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <src/stb_image.h>
#include <src/stb_image_write.h>

// init module
Renderer::Renderer(EasyVisConfig *config, CameraPose *pose){
    //OpenGL initialize
    setupOpenGL();
    TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    glGenBuffers(1, &vertexbuffer);
    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer); 
	  glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);

    //load texture
    generateTextureMap(config);
    Texture = loadBMP_custom(config->BACKGROUND_TEXTURE_PATH_BMP.c_str());

    InitializeViewParams(pose);

    cameraPose=pose;

    // set control values
    move_increamental = 0.1;
	  rotation_increamental = 0.3;
	  eye_dist = 1;
	  stereo = false;
    useForeground = true;
    x = 0;
    y = 0;
    z = 0;
    yaw = 0; //in radians
    pitch = 0;
    roll = 0;    

    Model = glm::mat4(1.0f);
    t = glm::mat4(1.0f);
    rx = glm::mat3(1.0f);
    ry = glm::mat3(1.0f);
    rz = glm::mat3(1.0f);

    // Set view center
    centering=glm::translate(glm::mat4(), glm::vec3(cameraPose->Ts[0].at<double>(0,0), cameraPose->Ts[0].at<double>(0,1), cameraPose->Ts[0].at<double>(0,2)));
	  offcenter=glm::translate(glm::mat4(), glm::vec3(-cameraPose->Ts[0].at<double>(0,0), -cameraPose->Ts[0].at<double>(0,1), -cameraPose->Ts[0].at<double>(0,2)));

    Projection = createProjectionFromIntrinsics(689.113, 689.113, 320.0f, 240.0f, (float)640, (float)480,-2.5, 2.0);


    // 3D models initialization
    backgroundModel= new BackgroundModel(config, UVScale, UVOffset);
    grasper= new GrasperModel(config);
    bean= new BeanModel(config);

    // Set view center
    getBgCenter();

    backgroundSize=backgroundModel->Vertices.size();

    // initialize saver
    save=false;
    viewNumber=config->VIEW_NUMBER;

    setRefView=0;
}

Renderer::~Renderer(){}

// estimate bg center position
void Renderer::getBgCenter(){
    float xs=0.0,ys=0.0,zs=0.0;
    float cnt=0.0;
    for(auto &vertice:backgroundModel->Vertices){
        xs+=(float)vertice.x;
        ys+=(float)vertice.y;
        zs+=(float)vertice.z;

        cnt++;
    }

    xs/=cnt;
    ys/=cnt;
    zs/=cnt;

    bgCenter=cv::Point3f(xs,ys,zs);
}

// each frame, update render param
void Renderer::RenderUpdateParam(){
    t[3].x = x;
    t[3].y = y;
    t[3].z = z;

    //Rotation around X
    rx[1].y = glm::cos(pitch);
    rx[1].z = glm::sin(pitch)*(-1);
    rx[2].y = glm::sin(pitch);
    rx[2].z = glm::cos(pitch);

    //Rotation around Y
    ry[0].x = glm::cos(yaw);
    ry[0].z = glm::sin(yaw);
    ry[2].x = glm::sin(yaw)*(-1);
    ry[2].z = glm::cos(yaw);

    //Rotation around Z
    rz[0].x = glm::cos(roll);
    rz[0].y = glm::sin(roll)*(-1);
    rz[1].x = glm::sin(roll);
    rz[1].y = glm::cos(roll);
}

// manual control
void Renderer::RenderControl(){

    double move_increamental = 0.1;
    double rotation_increamental = 0.1;
    cv::Mat xRoPi = (cv::Mat_<double>(3,3)<<1,0,0,0,-1,0,0,0,-1);
    std::vector<cv::Mat> Rs_est = cameraPose->Rs;
    std::vector<cv::Mat> ts_est = cameraPose->Ts;

    glm::vec3 forward;
    forward.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    forward.y = sin(glm::radians(pitch));
    forward.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    forward = glm::normalize(forward);

    glm::vec3 right;
    right.x = sin(glm::radians(yaw));
    right.y = 0;
    right.z = -cos(glm::radians(yaw));
    right = glm::normalize(right);

    glm::vec3 up;
    up = glm::normalize(glm::cross(right, forward));
    up = -up;

    //enable stereo
    if (glfwGetKey( window, GLFW_KEY_O ) == GLFW_PRESS && stereo==true){
        stereo=false;
        glfwSetWindowSize(window,640,480);
    }

    if (glfwGetKey( window, GLFW_KEY_P ) == GLFW_PRESS){
        stereo=true;
        glfwSetWindowSize(window,1280,480);
    }
        
    //eye distance
    if (glfwGetKey( window, GLFW_KEY_9  ) == GLFW_PRESS){
        eye_dist+=0.001;
    }
    if (glfwGetKey( window, GLFW_KEY_0    ) == GLFW_PRESS){
        eye_dist-=0.001;
    }

    // Move up
    if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS){
        x-=up.x*move_increamental;
        y-=up.y*move_increamental;
        z-=up.z*move_increamental;
    }

    // Move down
    if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS){
        x+=up.x*move_increamental;
        y+=up.y*move_increamental;
        z+=up.z*move_increamental;
    }

    // move right
    if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
        x+=forward.x*move_increamental;
        y+=forward.y*move_increamental;
        z+=forward.z*move_increamental;
    }

    // move left
    if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS){
        
        x-=forward.x*move_increamental;
        y-=forward.y*move_increamental;
        z-=forward.z*move_increamental;
    }	

    // move forward
    if (glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS){
        x+=right.x*move_increamental;
        y+=right.y*move_increamental;
        z+=right.z*move_increamental;
    }

    //move backward
    if (glfwGetKey( window, GLFW_KEY_S) == GLFW_PRESS){
        x-=right.x*move_increamental;
        y-=right.y*move_increamental;
        z-=right.z*move_increamental;
    }	

    // //move forward
    // if (glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS){
    //     z+=move_increamental;
    // }

    // //move backward
    // if (glfwGetKey( window, GLFW_KEY_S) == GLFW_PRESS){
    //     z-=move_increamental;
    // }	 	 	
    
    //pitch
    if (glfwGetKey( window, GLFW_KEY_X) == GLFW_PRESS){
        pitch+=rotation_increamental;
    }

    if (glfwGetKey( window, GLFW_KEY_C) == GLFW_PRESS){
        pitch-=rotation_increamental;
    }

    //yaw
    if (glfwGetKey( window, GLFW_KEY_Y) == GLFW_PRESS){
        yaw+=rotation_increamental;
    }

    if (glfwGetKey( window, GLFW_KEY_U) == GLFW_PRESS){
        yaw-=rotation_increamental;
    }

    //roll
    if (glfwGetKey( window, GLFW_KEY_Z) == GLFW_PRESS){
        roll+=rotation_increamental;
    }

    if (glfwGetKey( window, GLFW_KEY_A) == GLFW_PRESS){
        roll-=rotation_increamental;
    }

    if (glfwGetKey( window, GLFW_KEY_I) == GLFW_PRESS){
        save=true;
    }

    if (glfwGetKey( window, GLFW_KEY_J) == GLFW_PRESS){
        save=false;
    }

    cv::Mat R_tran;
    //------------------------------------------------------------------------------------
    //camera1 
    //press 1, set to viewpoint of camera1
    if (glfwGetKey( window, GLFW_KEY_1) == GLFW_PRESS){
        setRefView=0;

        x = 0;
        y = 0;
        z = 0;
        yaw = 0; //in radians
        pitch = 0;
        roll = 0;    
    }
    //camera2
    //press 2, set to viewpoint of camera2
    if (glfwGetKey( window, GLFW_KEY_2) == GLFW_PRESS){
        setRefView=1;

        x = 0;
        y = 0;
        z = 0;
        yaw = 0; //in radians
        pitch = 0;
        roll = 0;   
    }
    //camera3
    //press 3, set to viewpoint of camera3
    if (glfwGetKey( window, GLFW_KEY_3) == GLFW_PRESS){
        setRefView=2;

        x = 0;
        y = 0;
        z = 0;
        yaw = 0; //in radians
        pitch = 0;
        roll = 0;   
    }
    //camera4
    //press 4, set to viewpoint of camera4
    if (glfwGetKey( window, GLFW_KEY_4) == GLFW_PRESS){
        setRefView=3;

        x = 0;
        y = 0;
        z = 0;
        yaw = 0; //in radians
        pitch = 0;
        roll = 0;   
    }

    //camera5
    //press 5, set to viewpoint of camera5
    if (glfwGetKey( window, GLFW_KEY_5) == GLFW_PRESS){
        setRefView=4;

        x = 0;
        y = 0;
        z = 0;
        yaw = 0; //in radians
        pitch = 0;
        roll = 0;   
    }	

    //camera5
    //press 6, set to viewpoint of camera4
    if (glfwGetKey( window, GLFW_KEY_6) == GLFW_PRESS){
        View[0].x = 0.616378; //first column
        View[0].y = 0.649176;
        View[0].z = 0.445700;
        View[0].w = 0; 

        View[1].x = 0.583939; //second column
        View[1].y = -0.756544;
        View[1].z = 0.294376;
        View[1].w = 0;

        View[2].x = 0.528293; //third column
        View[2].y = 0.0788144;
        View[2].z = -0.845396;
        View[2].w = 0;

        // fourth column
        View[3].x = -1.27141;
        View[3].y = -0.523748;
        View[3].z = -0.992603;
        View[3].w = 1;
    }   

    //reset
    if (glfwGetKey( window, GLFW_KEY_R) == GLFW_PRESS){
        pitch=-atan2(cameraPose->Rs[0].at<double>(2,1),cameraPose->Rs[0].at<double>(2,2));
        yaw=-atan2(-cameraPose->Rs[0].at<double>(2,0),sqrt((pow(cameraPose->Rs[0].at<double>(0,0),2))+(pow(cameraPose->Rs[0].at<double>(1,2),2))));
        roll=-atan2(cameraPose->Rs[0].at<double>(1,0),cameraPose->Rs[0].at<double>(0,0));

        x=-cameraPose->Ts[0].at<double>(0,0);
        y=-cameraPose->Ts[0].at<double>(0,1);
        z=-cameraPose->Ts[0].at<double>(0,2);
    }

    //foreground update
    if (glfwGetKey( window, GLFW_KEY_N ) == GLFW_PRESS){
        useForeground = true;
    }
    if (glfwGetKey( window, GLFW_KEY_M ) == GLFW_PRESS){
        useForeground = false;
    }

    RenderUpdateParam();
}

void Renderer::reset(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(programID);
}

// flush buffer
void Renderer::flash(){
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    // Swap buffers
    glfwSwapBuffers(window);
    glfwPollEvents();
}

void Renderer::InitializeViewParams(CameraPose *pose){
    cv::Mat xRoPi = (cv::Mat_<double>(3,3)<<1,0,0,0,-1,0,0,0,-1);

    //Default viewing angle
    Rs_rec = xRoPi * pose->Rs[0];
    T1 = xRoPi * pose->Ts[0];

    //R
    View[0].x = (float) (Rs_rec.at<double>(0,0) ); //first column
    View[0].y = (float) (Rs_rec.at<double>(1,0) );
    View[0].z = (float) (Rs_rec.at<double>(2,0) );
    View[0].w = 0; 
        
    View[1].x = (float) (Rs_rec.at<double>(0,1) ); //second column
    View[1].y = (float) (Rs_rec.at<double>(1,1) );
    View[1].z = (float) (Rs_rec.at<double>(2,1) );
    View[1].w = 0;
        
    View[2].x = (float) (Rs_rec.at<double>(0,2) ); //third column
    View[2].y = (float) (Rs_rec.at<double>(1,2) );
    View[2].z = (float) (Rs_rec.at<double>(2,2) );
    View[2].w = 0;

    View[3].x = (float) (T1.at<double>(0,0) );
    View[3].y = (float) (T1.at<double>(0,1) );
    View[3].z = (float) (T1.at<double>(0,2) );
    View[3].w = 1.0f;	
}

// generate texture map for models from multiple color images
void Renderer::generateTextureMap(EasyVisConfig *config){
    cv::Mat VM;

    cv::Mat mvsImg = cv::imread(config->BACKGROUND_TEXTURE_PATH_JPG, -1);
    cv::Mat rodColor=cv::imread("./color_data/black.jpg");
    cv::Mat beanColor = cv::imread("./color_data/bean.png");
    cv::Mat blueColor = cv::imread("./color_data/b.png");
    cv::Mat fingerColor = cv::imread("./color_data/gray.png");

    cv::resize(rodColor, rodColor, cv::Size(mvsImg.cols, mvsImg.rows), cv::INTER_LINEAR);  
    cv::resize(beanColor, beanColor, cv::Size(mvsImg.cols, mvsImg.rows), cv::INTER_LINEAR);  
    cv::resize(blueColor, blueColor, cv::Size(mvsImg.cols, mvsImg.rows), cv::INTER_LINEAR); 
    cv::resize(fingerColor, fingerColor, cv::Size(mvsImg.cols, mvsImg.rows), cv::INTER_LINEAR);  

    cv::cvtColor(rodColor, rodColor, cv::COLOR_BGRA2BGR);
    cv::cvtColor(beanColor, beanColor, cv::COLOR_BGRA2BGR);
    cv::cvtColor(blueColor, blueColor, cv::COLOR_BGRA2BGR);
    cv::cvtColor(fingerColor, fingerColor, cv::COLOR_BGRA2BGR);

    vconcat(rodColor,beanColor,VM);
    vconcat(VM,blueColor,VM);
    vconcat(VM,fingerColor,VM);

    int newWidth = VM.cols > mvsImg.cols ? VM.cols : mvsImg.cols;
    int newHeight = VM.rows + mvsImg.rows;
    
    cv::Mat newTexture = cv::Mat::zeros(newHeight, newWidth, CV_8UC3);
    
    cv::Mat insetImage1(newTexture, cv::Rect(0, 0, mvsImg.cols, mvsImg.rows));
    mvsImg.copyTo(insetImage1);
    cv::Mat insetImage2(newTexture, cv::Rect(0, mvsImg.rows, VM.cols, VM.rows));
    VM.copyTo(insetImage2);
    cv::imwrite(config->BACKGROUND_TEXTURE_PATH_BMP, newTexture);

    UVScale=double(mvsImg.rows)/double(newTexture.rows);
    UVOffset=double(VM.rows)/double(newTexture.rows);
}

void Renderer::setupOpenGL(){
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(1920, 1080, "Anyview_stereo", NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        getchar();
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  
    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, 1024/2, 768/2);

    // background color
    glClearColor(0.5725f, 0.5725f, 0.5725f, 0.0f);


    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS); 

    // Cull triangles which normal is not towards the camera
    //glEnable(GL_CULL_FACE);

    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    // Create and compile our GLSL program from the shaders
    programID = LoadShaders( "./data/TransformVertexShader.vertexshader", "./data/TextureFragmentShader.fragmentshader" );
    // Get a handle for our "MVP" uniform
    MatrixID = glGetUniformLocation(programID, "MVP");
}

// generate render projection matrix
glm::mat4 Renderer::createProjectionFromIntrinsics(
    float fx, float fy,  // focal lengths
    float cx, float cy,  // principal point
    float width, float height,  // image dimensions
    float nearZ, float farZ    // near and far clipping planes
) {
    // Create projection matrix that matches the intrinsic parameters
    glm::mat4 projection = glm::mat4(0.0f);
    
    // Note: The signs are adjusted to match OpenGL's coordinate system
    projection[0][0] = 0.02*fx / (farZ - nearZ);
    projection[1][1] = 0.02*fy / (farZ - nearZ);
    
    // Principal point offset (normalized to [-1, 1])
    projection[0][2] = 2.0f * (cx / width) - 1.0f;
    projection[1][2] = 2.0f * (cy / height) - 1.0f;
    projection[2][2] = 0.1f;
    projection[2][3] = 1.8f;
    
    projection[3][2] = -1.0f;
    
    return projection;
}

// run renderer, monocular or stereo can be choosen
void Renderer::Render(std::vector<glm::vec3> &Vertices, std::vector<glm::vec2> &Uvs){
    if(!stereo){
        // single view
        // glViewport(0, 0, 640, 480);
        glViewport(0, 0, 1920, 1080);
        // glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 640.0f / 480.0f, 0.1f, 100.0f);
        // glm::mat4 MVP = Projection * t * offcenter * rz * rx * ry * centering * View * Model;


        glm::mat4 cvToGL = glm::mat4(
            -1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1
        );

        glm::mat4 model = glm::mat4(1.0f);  // Identity model matrix

        glm::mat4 view = glm::mat4(1.0f);

        glm::mat3 R = glm::mat3(1.0f);
        glm::vec3 T = glm::vec3(1.0f);

        T[0]=cameraPose->Ts[setRefView].at<double>(0,0);
        T[1]=-cameraPose->Ts[setRefView].at<double>(0,1);
        T[2]=cameraPose->Ts[setRefView].at<double>(0,2);


        
    
        // Set rotation part (transpose of R since OpenGL uses column-major)
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                R[i][j] = cameraPose->Rs[setRefView].at<double>(i,j);
            }
        }

        R=R*rz * rx * ry;

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                view[i][j] = R[j][i];            
            }
        }
        
        // Set translation part (-R^T * t)
        glm::vec3 trans = -glm::transpose(R) * T;
        view[3][0] = trans.x;
        view[3][1] = trans.y;
        view[3][2] = trans.z;

        // glm::mat4 MVP = Projection * view * model;
        glm::mat4 MVP =cvToGL*Projection* t* view;

        //glm::mat4 inter = t * offcenter * rz * rx * ry * centering * View;
        // Set our "myTextureSampler" sampler to use Texture Unit 0
        glUniform1i(TextureID, 0);
        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
            0,                  // attribute
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
            1,                                // attribute
            2,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            0,                                // stride
            (void*)0                          // array buffer offset
        );

        


        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, Vertices.size() * sizeof(glm::vec3), &Vertices[0], GL_STATIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glBufferData(GL_ARRAY_BUFFER, Uvs.size() * sizeof(glm::vec2), &Uvs[0], GL_STATIC_DRAW);

        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        glDrawArrays(GL_TRIANGLES, 0, Vertices.size() );
    }
    else{
        // stereo
        //draw objects
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, Vertices.size() * sizeof(glm::vec3), &Vertices[0], GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glBufferData(GL_ARRAY_BUFFER, Uvs.size() * sizeof(glm::vec2), &Uvs[0], GL_STATIC_DRAW);

        //draw the left eye, left half of the screen
        glViewport(0, 0, 640, 480);
        glm::mat4 l_View = View; //construct identity matrix
        l_View[3].x = (float) (l_View[3].x - 0.1 );
        // glm::mat4 l_MVP = Projection * t * offcenter * rz * rx * ry * centering * l_View * Model;

        glm::mat4 l_MVP=Projection;
        // Send our transformation to the currently bound shader, 
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &l_MVP[0][0]);
        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        // Set our "myTextureSampler" sampler to use Texture Unit 0
        glUniform1i(TextureID, 0);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
            0,                  // attribute
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
            1,                                // attribute
            2,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            0,                                // stride
            (void*)0                          // array buffer offset
        );

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, Vertices.size() );

        //Draw the right eye
        //draw the right eye, right half of the screen
        glViewport(640, 0, 640, 480);
        glm::mat4 r_View = View; //construct identity matrix
        r_View[3].x = (float) (r_View[3].x + 0.1 );
        // glm::mat4 r_MVP = Projection * t * offcenter * rz * rx * ry * centering * r_View * Model;

        glm::mat4 r_MVP = glm::mat4(1.0f);

        glm::mat3 R = glm::mat3(1.0f);
        glm::vec3 t = glm::vec3(1.0f);

        t[0]=cameraPose->Ts[0].at<double>(0,0);
        t[1]=cameraPose->Ts[0].at<double>(0,1);
        t[2]=cameraPose->Ts[0].at<double>(0,2);
    
        // Set rotation part (transpose of R since OpenGL uses column-major)
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                r_MVP[i][j] = cameraPose->Rs[0].at<double>(i,j);

                R[i][j] = cameraPose->Rs[0].at<double>(i,j);
            }
        }
        
        // Set translation part (-R^T * t)
        glm::vec3 trans = -glm::transpose(R) * t;
        r_MVP[3][0] = trans.x;
        r_MVP[3][1] = trans.y;
        r_MVP[3][2] = trans.z;

        // Send our transformation to the currently bound shader, 
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &r_MVP[0][0]);
        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        // Set our "myTextureSampler" sampler to use Texture Unit 0
        glUniform1i(TextureID, 0);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
            0,                  // attribute
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
            1,                                // attribute
            2,                                // size
            GL_FLOAT,                         // type
            GL_FALSE,                         // normalized?
            0,                                // stride
            (void*)0                          // array buffer offset
        );

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, Vertices.size());	
    }
}

// final clean when end program
void Renderer::clean(){
    // Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &uvbuffer);
	glDeleteProgram(programID);
	//glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

    // Closes all the frames
    cv::destroyAllWindows();
}

// 3D model transfer according to 3D pose
std::vector<glm::vec3> Renderer::transferModelFinger(   
        std::vector<glm::vec3> finger, 
        double rotation, double angle){ 

    //rotation=rotation*2+0.5*M_PI;
    
    //rotate y axis
    std::vector<std::vector<double>> rot_x = {  {1, 0, 0}, 
                                                {0, cos(angle), -sin(angle)},
                                                {0, sin(angle), cos(angle)}};                                                         
    std::vector<std::vector<double>> rot_y = {  {cos(rotation), 0, sin(rotation)},
                                                {0, 1, 0},
                                                {-sin(rotation), 0, cos(rotation)}};  
    std::vector<std::vector<double>> rot_z = {  {cos(rotation), -sin(rotation), 0},
                                                {sin(rotation), cos(rotation), 0},
                                                {0, 0, 1}};   
                                             
    // two stage finger rotation along x and z axis                
    std::vector<glm::vec3> newFinger(finger.size());  
    for (int l = 0; l < finger.size(); l++){
        newFinger[l] = matrixMul(finger[l], rot_x);
        newFinger[l] = matrixMul(newFinger[l], rot_z);
    }
    
    return newFinger;
}

// mat mul
glm::vec3 Renderer::matrixMul(glm::vec3 points, std::vector<std::vector<double>> rot){
    glm::vec3 output;
    output.x = points.x * rot[0][0]+ points.y * rot[1][0] + points.z * rot[2][0];
    output.y = points.x * rot[0][1]+ points.y * rot[1][1] + points.z * rot[2][1];
    output.z = points.x * rot[0][2]+ points.y * rot[1][2] + points.z * rot[2][2];
    return output;
}

// transfer 3D model according to 3D pose
std::vector<glm::vec3> Renderer::transferModel(
        std::vector<glm::vec3> model, 
        std::vector<float> direction, 
        std::vector<float> translation,
        float offset){

    float d1 = direction[0], d2 = direction[1], d3 = direction[2];
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
                                

    // Rotation and translation
    std::vector<glm::vec3> newModel(model.size());   
    for (int i=0;i<model.size();i++){
        newModel[i].x =  model[i].x * rot[0][0]+ model[i].y * rot[1][0] + model[i].z * rot[2][0]+translation[0]-direction[0]*offset;
        newModel[i].y =  model[i].x * rot[0][1]+ model[i].y * rot[1][1] + model[i].z * rot[2][1]+translation[1]-direction[1]*offset;
        newModel[i].z =  model[i].x * rot[0][2]+ model[i].y * rot[1][2] + model[i].z * rot[2][2]+translation[2]-direction[2]*offset;
    }

    return newModel;
}

// 3D rendering without KFs
void Renderer::runNoTrack(
        std::vector<grasper3d> graspersReconstructed,
        std::vector<bean3d> beansReconstructed,
        std::vector<cv::Mat> imgs){
    
    // tmp disable bg for ppr show
    std::vector<glm::vec3> RenderVertices(backgroundModel->Vertices);
    std::vector<glm::vec2> RenderUvs(backgroundModel->Uvs);

    std::vector<float> zeroVector={0.0f,0.0f,0.0f};
    for(auto& beans:beansReconstructed){
        // 0 here is the offset
        std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, {beans.beanCenter.x,beans.beanCenter.y,beans.beanCenter.z}, 0);
        RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
        RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        
    }
    
    reset();
    RenderControl();
    Render(RenderVertices,RenderUvs);
    flash();

    saveNovelView(imgs,RenderVertices,RenderUvs);
}

// run renderer
void Renderer::run(
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
        std::vector<cv::Mat> imgs){
    
    // tmp disable bg for ppr show
    std::vector<glm::vec3> RenderVertices(backgroundModel->Vertices);
    std::vector<glm::vec2> RenderUvs(backgroundModel->Uvs);

    // Need loop due to multiple grasper
    for(auto id:grasper_B_Ids){
        if(graspers_B_KF3d.find(id)!=graspers_B_KF3d.end() &&  graspers_B_KF3d.at(id).valid){
            std::vector<glm::vec3> fingerLeftVertice = transferModelFinger(grasper->fingerLeft.Vertices, graspers_B_KF3d.at(id).rotation.output, graspers_B_KF3d.at(id).angle.output);
            std::vector<glm::vec3> fingerRightVertice = transferModelFinger(grasper->fingerRight.Vertices, graspers_B_KF3d.at(id).rotation.output, -graspers_B_KF3d.at(id).angle.output);
            std::vector<glm::vec3> grasperVertice(grasper->rod.Vertices);
            grasperVertice.insert(grasperVertice.end(),fingerLeftVertice.begin(),fingerLeftVertice.end());
            grasperVertice.insert(grasperVertice.end(),fingerRightVertice.begin(),fingerRightVertice.end());

            // 0.2 is the offset, need to move it
            grasperVertice = transferModel(grasperVertice, graspers_B_KF3d.at(id).directionKF.output, graspers_B_KF3d.at(id).jointKF.output, 0.15);
            RenderVertices.insert(RenderVertices.end(),grasperVertice.begin(),grasperVertice.end());

            RenderUvs.insert(RenderUvs.end(),grasper->rod.Uvs.begin(),grasper->rod.Uvs.end());
            RenderUvs.insert(RenderUvs.end(),grasper->fingerLeft.Uvs.begin(),grasper->fingerLeft.Uvs.end());
            RenderUvs.insert(RenderUvs.end(),grasper->fingerRight.Uvs.begin(),grasper->fingerRight.Uvs.end());
        }
    }
    for(auto id:grasper_W_Ids){
        if(graspers_W_KF3d.find(id)!=graspers_W_KF3d.end() &&  graspers_W_KF3d.at(id).valid){
            std::vector<glm::vec3> fingerLeftVertice = transferModelFinger(grasper->fingerLeft.Vertices, graspers_W_KF3d.at(id).rotation.output, graspers_W_KF3d.at(id).angle.output);
            std::vector<glm::vec3> fingerRightVertice = transferModelFinger(grasper->fingerRight.Vertices, graspers_W_KF3d.at(id).rotation.output, -graspers_W_KF3d.at(id).angle.output);
            std::vector<glm::vec3> grasperVertice(grasper->rod.Vertices);
            grasperVertice.insert(grasperVertice.end(),fingerLeftVertice.begin(),fingerLeftVertice.end());
            grasperVertice.insert(grasperVertice.end(),fingerRightVertice.begin(),fingerRightVertice.end());

            // 0.2 is the offset, need to move it
            grasperVertice = transferModel(grasperVertice, graspers_W_KF3d.at(id).directionKF.output, graspers_W_KF3d.at(id).jointKF.output, 0.15);
            RenderVertices.insert(RenderVertices.end(),grasperVertice.begin(),grasperVertice.end());

            RenderUvs.insert(RenderUvs.end(),grasper->rod.Uvs.begin(),grasper->rod.Uvs.end());
            RenderUvs.insert(RenderUvs.end(),grasper->fingerLeft.Uvs.begin(),grasper->fingerLeft.Uvs.end());
            RenderUvs.insert(RenderUvs.end(),grasper->fingerRight.Uvs.begin(),grasper->fingerRight.Uvs.end());
        }
    }

    std::vector<float> zeroVector={0.0f,0.0f,0.0f};
    for(auto id:bean_blk_Ids){
        // 0 here is the offset
        if(beans_blk_KF3d.find(id)!=beans_blk_KF3d.end() && beans_blk_KF3d.at(id).valid){
            std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, beans_blk_KF3d.at(id).beanCenterKF.output, 0);
            RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
            RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        }
        
    }
    for(auto id:bean_dkbl_Ids){
        // 0 here is the offset
        if(beans_dkbl_KF3d.find(id)!=beans_dkbl_KF3d.end() && beans_dkbl_KF3d.at(id).valid){
            std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, beans_dkbl_KF3d.at(id).beanCenterKF.output, 0);
            RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
            RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        }
        
    }
    for(auto id:bean_gray_Ids){
        // 0 here is the offset
        if(beans_gray_KF3d.find(id)!=beans_gray_KF3d.end() && beans_gray_KF3d.at(id).valid){
            std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, beans_gray_KF3d.at(id).beanCenterKF.output, 0);
            RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
            RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        }
        
    }
    for(auto id:bean_gren_Ids){
        // 0 here is the offset
        if(beans_gren_KF3d.find(id)!=beans_gren_KF3d.end() && beans_gren_KF3d.at(id).valid){
            std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, beans_gren_KF3d.at(id).beanCenterKF.output, 0);
            RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
            RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        }
        
    }
    for(auto id:bean_ltbl_Ids){
        // 0 here is the offset
        if(beans_ltbl_KF3d.find(id)!=beans_ltbl_KF3d.end() && beans_ltbl_KF3d.at(id).valid){
            std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, beans_ltbl_KF3d.at(id).beanCenterKF.output, 0);
            RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
            RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        }
        
    }
    for(auto id:bean_orng_Ids){
        // 0 here is the offset
        if(beans_orng_KF3d.find(id)!=beans_orng_KF3d.end() && beans_orng_KF3d.at(id).valid){
            std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, beans_orng_KF3d.at(id).beanCenterKF.output, 0);
            RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
            RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        }
        
    }
    for(auto id:bean_prpl_Ids){
        // 0 here is the offset
        if(beans_prpl_KF3d.find(id)!=beans_prpl_KF3d.end() && beans_prpl_KF3d.at(id).valid){
            std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, beans_prpl_KF3d.at(id).beanCenterKF.output, 0);
            RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
            RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        }
        
    }
    for(auto id:bean_red_Ids){
        // 0 here is the offset
        if(beans_red_KF3d.find(id)!=beans_red_KF3d.end() && beans_red_KF3d.at(id).valid){
            std::vector<glm::vec3> beanVertice=transferModel(bean->bean.Vertices, zeroVector, beans_red_KF3d.at(id).beanCenterKF.output, 0);
            RenderVertices.insert(RenderVertices.end(),beanVertice.begin(),beanVertice.end());
            RenderUvs.insert(RenderUvs.end(),bean->bean.Uvs.begin(),bean->bean.Uvs.end());
        }
        
    }
    
    reset();
    RenderControl();
    Render(RenderVertices,RenderUvs);
    flash();

    saveNovelView(imgs,RenderVertices,RenderUvs);
}

// show detection results
void Renderer::notice(
        std::vector<cv::Mat> imgs, 
        std::vector<std::vector<grasper2d>> graspers,
        std::vector<std::vector<bean2d>> beans){
    
    std::vector<cv::Mat> downSizeImgs(imgs.size());

    #pragma omp parallel for
    for(int i=0;i<imgs.size();i++){
        cv::resize(imgs[i], downSizeImgs[i], cv::Size(160, 120), cv::INTER_LINEAR);

        bool objectInBox=false;

        for(auto grasper:graspers[i]){
            if(grasper.locator.x>64 && grasper.locator.x<576 && grasper.locator.y>48 && grasper.locator.y<432){
                objectInBox=true;
            }
            else{
                objectInBox=false;
            }
        }

        for(auto bean:beans[i]){
            if(bean.beanCenter.x>64 && bean.beanCenter.x<576 && bean.beanCenter.y>48 && bean.beanCenter.y<432){
                objectInBox=true;
            }
            else{
                objectInBox=false;
            }
        }

        if(objectInBox){
            cv::rectangle(downSizeImgs[i], cv::Point(16,12), cv::Point(144,108), cv::Scalar(0, 255, 0), 2, cv::LINE_8); 
        }
        else{
            cv::rectangle(downSizeImgs[i], cv::Point(16,12), cv::Point(144,108), cv::Scalar(0, 0, 255), 2, cv::LINE_8); 
        }
    }

    cv::Mat allImg(downSizeImgs[0]);

    for(int i=1;i<downSizeImgs.size();i++){
        cv::vconcat(allImg,downSizeImgs[i],allImg);
    }

    cv::imshow("AllViews",allImg);

    // move window, 2000 is a value larger than screen width, just force it leftmost
    cv::moveWindow("AllViews", 2000,0);
    cv::waitKey(1);
}

// save rendered view
void Renderer::saveNovelView(
        std::vector<cv::Mat> groundTruths,
        std::vector<glm::vec3> &RenderVertices,
        std::vector<glm::vec2> &RenderUvs){

    if(!save){
        return;
    }

    if(savePath.size()==0){
        // create save path
        time_t _tm =time(NULL );
        struct tm * curtime = localtime ( &_tm );

        std::string date=asctime(curtime);
        // std::ostringstream oss;
        // oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
        // auto date = oss.str();
        
        savePath="./log/"+date+"/";
        mkdir(savePath.c_str(),0777);

        for(int i=0;i<viewNumber;i++){
            std::string savePathPerView=savePath+std::to_string(i);
            mkdir(savePathPerView.c_str(),0777);
        }

        saveCounter=0;
    }

    std::vector<cv::Mat> Rs_est = cameraPose->Rs;
    std::vector<cv::Mat> ts_est = cameraPose->Ts;

    glm::mat4 viewBackup=View;

    for (int kk = 0; kk < viewNumber; kk++){
        reset();
        cv::Mat xRoPi = (cv::Mat_<double>(3,3)<<1,0,0,0,-1,0,0,0,-1);
        cv::Mat Rs_rec_temp = xRoPi * Rs_est[kk];
        cv::Mat TT= xRoPi*ts_est[kk];
        glm::mat4 V(1.0f);
        
        View[0].x = (float) (Rs_rec_temp.at<double>(0,0) ); //first column
        View[0].y = (float) (Rs_rec_temp.at<double>(1,0) );
        View[0].z = (float) (Rs_rec_temp.at<double>(2,0) );
        View[0].w = 0; 

        View[1].x = (float) (Rs_rec_temp.at<double>(0,1) ); //second column
        View[1].y = (float) (Rs_rec_temp.at<double>(1,1) );
        View[1].z = (float) (Rs_rec_temp.at<double>(2,1) );
        View[1].w = 0;

        View[2].x = (float) (Rs_rec_temp.at<double>(0,2) ); //third column
        View[2].y = (float) (Rs_rec_temp.at<double>(1,2) );
        View[2].z = (float) (Rs_rec_temp.at<double>(2,2) );
        View[2].w = 0;

        // fourth column
        View[3].x = (float) (TT.at<double>(0,0) );
        View[3].y = (float) (TT.at<double>(0,1) );
        View[3].z = (float) (TT.at<double>(0,2) );

        //reset();
        Render(RenderVertices,RenderUvs);
        flash();
        
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        GLsizei nrChannels = 3;
        GLsizei stride = nrChannels * width;
        stride += (stride % 4) ? (4 - stride % 4) : 0;
        GLsizei bufferSize = stride * height;
        std::vector<char> buffer(bufferSize);
        glPixelStorei(GL_PACK_ALIGNMENT, 4);
        //glReadBuffer(GL_FRONT);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
        stbi_flip_vertically_on_write(true);

        std::string pathSaveRendered=savePath+std::to_string(kk)+"/rendered"+std::to_string(saveCounter)+".png";
        std::string pathSaveGt=savePath+std::to_string(kk)+"/gt"+std::to_string(saveCounter)+".png";


        stbi_write_png(pathSaveRendered.c_str(), width, height, nrChannels, buffer.data(), stride);
        imwrite(pathSaveGt,groundTruths[kk]);

        glfwSwapBuffers(window);
        glfwPollEvents();

        //flash();
    }

    saveCounter++;
    View=viewBackup;
}
