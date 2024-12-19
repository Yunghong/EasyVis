cmake_minimum_required(VERSION 3.25)
project(EasyVis)

# Use ccache to speed up rebuilds
#include(cmake/ccache.cmake)

# Set C++ version and optimization level
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations -fopenmp")

# CMake Options
option(ENABLE_BENCHMARKS "Benchmark the preprocessing, inference, and post processing" OFF)
if (ENABLE_BENCHMARKS)
    add_compile_definitions(ENABLE_BENCHMARKS)
endif()

find_package(OpenCV REQUIRED)


# The apt install OpenGL have problem
#find_package(glfw3 REQUIRED)
#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)
#find_package(glm REQUIRED)
#find_package(GLEW REQUIRED)
#find_package(GLU REQUIRED)

#if(!GLEW_FOUND)
#    message( "glew not found" )
#endif()

add_subdirectory (external)

if(INCLUDE_DISTRIB)
	add_subdirectory(distrib)
endif(INCLUDE_DISTRIB)

include_directories(
	external/AntTweakBar-1.16/include/
	external/glfw-3.1.2/include/
	external/glm-0.9.7.1/
	external/glew-1.13.0/include/
	external/assimp-3.0.1270/include/
	external/bullet-2.81-rev2613/src/
	.
)

set(GL_LIBS
	${OPENGL_LIBRARY}
	glfw
	GLEW_1130
)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)

# TODO: Specify the path to TensorRT root dir
set(TensorRT_DIR /usr/src/tensorrt)

# Build the TensorRT inference engine library
# TensorRT is found and linked by the tensorrt-cpp-api
add_subdirectory(libs/tensorrt-cpp-api)

# Build the YoloV8 library
add_library(YoloV8_TRT SHARED src/yolov8.cpp)
target_link_libraries(YoloV8_TRT PUBLIC tensorrt_cpp_api ${OpenCV_LIBS})
target_include_directories(YoloV8_TRT PUBLIC libs/tensorrt-cpp-api/src)

# Build Eigen
set(EIGEN_VERSION 3.3)
find_package(Eigen3 ${EIGEN_VERSION} QUIET)
if(NOT EIGEN3_FOUND)
  set(BUILD_TESTING OFF CACHE INTERNAL "")
  FetchContent_Declare(eigen
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         ${EIGEN_VERSION}
    GIT_SHALLOW     ON)
  FetchContent_MakeAvailable(eigen)
  unset(BUILD_TESTING CACHE)
endif()

# Build ByteTrack
#include_directories(bytetrack/include)
add_library(ByteTrack SHARED
    ByteTrack/src/BYTETracker.cpp
    ByteTrack/src/kalmanFilter.cpp
    ByteTrack/src/kalmanFilterBean.cpp
    ByteTrack/src/kalmanFilterBean3D.cpp
    ByteTrack/src/kalmanFilterGrasper.cpp
    ByteTrack/src/lapjv.cpp
    ByteTrack/src/utils.cpp
    ByteTrack/src/STrack.cpp
  )
target_include_directories(ByteTrack PUBLIC ByteTrack/include)
target_link_libraries(ByteTrack cudart nvinfer ${OpenCV_LIBS} Eigen3::Eigen)
# add_library(ByteTrack SHARED
#     ByteTrack/src/BYTETracker.cpp
#     ByteTrack/src/KalmanFilter.cpp
#     ByteTrack/src/lapjv.cpp
#     ByteTrack/src/Object.cpp
#     ByteTrack/src/Rect.cpp
#     ByteTrack/src/STrack.cpp
#   )
# target_include_directories(ByteTrack PUBLIC ByteTrack/include)
# target_link_libraries(ByteTrack PUBLIC Eigen3::Eigen)

# Build and link the executables
add_executable( EasyVisV3
                src/easyvis_v3.cpp
                src/camera.cpp
                src/camera.hpp
                src/config.hpp
                src/renderer.cpp
                src/renderer.hpp
                src/glmodel.cpp
                src/glmodel.hpp
                src/pose.cpp
                src/pose.hpp
                src/detection.cpp
                src/detection.hpp
                src/container.hpp
                src/kalmanfilter.cpp
                src/kalmanfilter.hpp
                src/reconstruction.cpp
                src/reconstruction.hpp
                src/validation.cpp
                src/validation.hpp
                src/epipolarTracking.hpp
                src/epipolarTracking.cpp
                src/util.hpp
                
                src/shader.cpp
                src/shader.hpp 
                src/shader.cpp
                src/shader.hpp
                src/texture.cpp
                src/texture.hpp
                src/objloader.cpp
                src/objloader.hpp
                src/lodepng.cpp
                src/lodepng.h
                src/happly.h
                src/stb_image_write.h
                src/stb_image.h)
                
SOURCE_GROUP(src REGULAR_EXPRESSION ".*/src/.*" )

target_link_libraries(EasyVisV3
	${GL_LIBS}
    YoloV8_TRT
    ByteTrack
)
#target_link_libraries(EasyVisV3 YoloV8_TRT OpenGL::GL GLUT::GLUT glfw GLEW::GLEW GLU)
