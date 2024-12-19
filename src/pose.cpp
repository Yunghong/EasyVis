// load camera pose. The pose describes 3D info of cameras in the 3D space

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <opencv2/calib3d.hpp>

#include "pose.hpp"

CameraPose::CameraPose(EasyVisConfig* config){
    //Initial camera positions
    for(int i=0;i<config->VIEW_NUMBER;i++){
        std::vector<std::vector<double>> P = readProjectionMatrix(config->DATA_ROOT+"/P"+std::to_string(i+1)+".txt");
        projMatrices.push_back((cv::Mat_<double>(3,4)<< P[0][0], P[0][1], P[0][2], P[0][3],
                                                        P[1][0], P[1][1], P[1][2], P[1][3],
                                                        P[2][0], P[2][1], P[2][2], P[2][3]));
    }
    
    //decompose Projection Matrices to get K, R, C, T
    DecomposeProjectionMatrices();  
	SaveKRts(config->DATA_ROOT);

}

std::vector<std::vector<double>> CameraPose::readProjectionMatrix(std::string fileName){
	std::vector<std::vector<double>> data;
	std::ifstream file(fileName);
	std::string line;
	while(std::getline(file, line)){
		std::vector<double> lineData;
		std::stringstream lineStream(line);
		double value;
		while(lineStream >> value){
			lineData.push_back(value);
		}
		data.push_back(lineData);
	}
	return data;
}

/*
* Function: DecomposeProjectionMatrices
* --------------------------------------
* Decompose Projection matrices to get intrisinc (K), Rotation (R), and Translation(T). 
*
* Input:
* -Projection matries
*
* Output:
* -Camera intrinsic Matries
* -Rotation Matries
* -Translation Vectors
*
* Note: The reference of Ks,Rs,Ts are passed to the funciton. Their values are computed in the function.
*/
void CameraPose::DecomposeProjectionMatrices(){
	// for(int i=0;i<4;i++){
    //     std::vector<cv::Mat> tmp;
    //     cameraMatrices.push_back(tmp);
    // }
    // for (int i=0;i<projMatrices.size();i++){
    //     cv::Mat P=projMatrices[i];
    for(auto P:projMatrices){

		cv::Mat K(3,3,cv::DataType<double>::type); // intrinsic matrix
		cv::Mat R(3,3,cv::DataType<double>::type); // rotation matrix
		cv::Mat C_Not_homogeneous(4,1,cv::DataType<double>::type); // Camera position(Homogeneous)
		cv::Mat C(3,1,cv::DataType<double>::type); // Camera position(Euclidian)
		cv::Mat T(3,1,cv::DataType<double>::type); // Translation Vector

		// Decompose Projection matrix
		cv::decomposeProjectionMatrix(P, K, R, C_Not_homogeneous);

		// Conver points from Homomgeneous to Euclidian distance
		// cv::convertPointsFromHomogeneous(Ch, C);
		C.at<double>(0,0) = C_Not_homogeneous.at<double>(0,0) / C_Not_homogeneous.at<double>(3,0);
		C.at<double>(1,0) = C_Not_homogeneous.at<double>(1,0) / C_Not_homogeneous.at<double>(3,0);
		C.at<double>(2,0) = C_Not_homogeneous.at<double>(2,0) / C_Not_homogeneous.at<double>(3,0);

		// Compute translation vector
		T = -1*R*C;

		Ks.push_back(K);
		Rs.push_back(R);
		Cs.push_back(C);
		Ts.push_back(T);
	}
}

void CameraPose::SaveKRts(std::string dataRoot){
	for(int i=0;i<Rs.size();i++){
		// std::ifstream file;
		// file.open(dataRoot+"/R"+std::to_string(i)+".txt");
		std::ofstream ofs(dataRoot+"/R"+std::to_string(i)+".txt", std::ofstream::trunc);

		ofs<<"[["<<Rs[i].at<double>(0,0)<<", "<< Rs[i].at<double>(0,1)<<", "<<Rs[i].at<double>(0,2)<<"], "<<std::endl;
		ofs<<" ["<<Rs[i].at<double>(1,0)<<", "<< Rs[i].at<double>(1,1)<<", "<<Rs[i].at<double>(1,2)<<"], "<<std::endl;
		ofs<<" ["<<Rs[i].at<double>(2,0)<<", "<< Rs[i].at<double>(2,1)<<", "<<Rs[i].at<double>(2,2)<<"]]"<<std::endl;

		ofs.close();
	}

	for(int i=0;i<Ts.size();i++){
		// std::ifstream file;
		// file.open(dataRoot+"/R"+std::to_string(i)+".txt");
		std::ofstream ofs(dataRoot+"/T"+std::to_string(i)+".txt", std::ofstream::trunc);

		ofs<<"[["<<Ts[i].at<double>(0,0)<<"], ";
		ofs<<" ["<<Ts[i].at<double>(1,0)<<"], ";
		ofs<<" ["<<Ts[i].at<double>(2,0)<<"]]"<<std::endl;

		ofs.close();
	}

	for(int i=0;i<Ks.size();i++){
		// std::ifstream file;
		// file.open(dataRoot+"/R"+std::to_string(i)+".txt");
		std::ofstream ofs(dataRoot+"/K"+std::to_string(i)+".txt", std::ofstream::trunc);

		ofs<<"[["<<Ks[i].at<double>(0,0)<<", "<< Ks[i].at<double>(0,1)<<", "<<Ks[i].at<double>(0,2)<<"], "<<std::endl;
		ofs<<" ["<<Ks[i].at<double>(1,0)<<", "<< Ks[i].at<double>(1,1)<<", "<<Ks[i].at<double>(1,2)<<"], "<<std::endl;
		ofs<<" ["<<Ks[i].at<double>(2,0)<<", "<< Ks[i].at<double>(2,1)<<", "<<Ks[i].at<double>(2,2)<<"]]"<<std::endl;

		ofs.close();
	}
}


CameraPose::~CameraPose(){}
