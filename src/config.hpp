#ifndef SETUP_H
#define SETUP_H

#include <string>
#include <vector>

// For parser
#include <iostream>
#include <fstream>
#include <sstream>

class EasyVisConfig{
public:
    std::string DATA_ROOT;
    std::vector<std::string> CAM_ADDRESSES;
    std::vector<int> CAM_PORTS;
    int CAM_RES_WIDTH;
    int CAM_RES_HEIGHT;
    int CAM_FPS;
    int MODEL_INPUT_WIDTH;
    int MODEL_INPUT_HEIGHT;
    int DISP_WIDTH;
    int DISP_HEIGHT;
    int VIEW_NUMBER;
    std::string MODEL_PATH;
    std::string BACKGROUND_TEXTURE_PATH_JPG;
    std::string BACKGROUND_TEXTURE_PATH_BMP;
    std::string BACKGROUND_MODEL_PATH;
    std::vector<double> FOREGROUND_SCALOR;
    
    // old version
    // EasyVisConfig(){
    //     // CAM_ADDRESSES={ "http://10.42.0.100:8000/?action=stream",
    //     //                 "http://10.42.0.101:8001/?action=stream",
    //     //                 "http://10.42.0.102:8002/?action=stream",
    //     //                 "http://10.42.0.103:8003/?action=stream",
    //     //                 "http://10.42.0.104:8004/?action=stream"};

      
    //     // VIEW_NUMBER=10;
    //     // CAM_PORTS={0, 2, 4, 6, 8, 10, 12, 14, 16, 18};

    //     VIEW_NUMBER=5;
    //     CAM_PORTS={2, 4, 8, 14, 18};

    //     // VIEW_NUMBER=10;
    //     // CAM_ADDRESSES={ "./data/test_videos/video1.mp4",
    //     //                 "./data/test_videos/video2.mp4",
    //     //                 "./data/test_videos/video3.mp4",
    //     //                 "./data/test_videos/video4.mp4",
    //     //                 "./data/test_videos/video5.mp4"};

    //     CAM_ADDRESSES={ "./data/test_videos/video1.mp4",
    //                     "./data/test_videos/video2.mp4",
    //                     "./data/test_videos/video3.mp4",
    //                     "./data/test_videos/video4.mp4",
    //                     "./data/test_videos/video5.mp4",
    //                     "./data/test_videos/video6.mp4",
    //                     "./data/test_videos/video7.mp4",
    //                     "./data/test_videos/video8.mp4",
    //                     "./data/test_videos/video9.mp4",
    //                     "./data/test_videos/video10.mp4"};
    //     CAM_RES_WIDTH=640;
    //     CAM_RES_HEIGHT=480;
    //     CAM_FPS=30;

    //     MODEL_INPUT_WIDTH=640;
    //     MODEL_INPUT_HEIGHT=480;

    //     DISP_WIDTH=640;
    //     DISP_HEIGHT=480;
        
    //     DATA_ROOT="./data";
    //     MODEL_PATH="./models/markerless.onnx";
    //     BACKGROUND_TEXTURE_PATH_JPG=DATA_ROOT+"/scene_dense_mesh_texture_material_0_map_Kd.jpg";
    //     BACKGROUND_TEXTURE_PATH_BMP=DATA_ROOT+"/scene_dense_meshtexture_material_0_map_Kd.bmp";
    //     BACKGROUND_MODEL_PATH=DATA_ROOT+"/scene_dense_mesh_texture.obj";

    //     // FOREGROUND_SCALOR={0.045,5,12};
    //     FOREGROUND_SCALOR={0.027,5,12};
        
        
    //     // VIEW_NUMBER=CAM_ADDRESSES.size();
                        
    // }
    
    // load config file
    EasyVisConfig(){
        loadFromFile("./config.txt");
    }

    void loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Could not open config file: " << filename << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream lineStream(line);
            std::string key;
            if (std::getline(lineStream, key, '=')) {
                std::string value;
                std::cout << key << std::endl;
                
                if (std::getline(lineStream, value)) {
                    setConfigValue(key, value);
                }
                std::cout << value << std::endl;
            }
        }
        file.close();
    }

private:
    void setConfigValue(const std::string& key, const std::string& value) {
        if (key == "DATA_ROOT") {
            DATA_ROOT = value;
        } else if (key == "CAM_ADDRESSES") {
            CAM_ADDRESSES = split(value, ',');
        } else if (key == "CAM_PORTS") {
            CAM_PORTS = parseInts(value);
        } else if (key == "CAM_RES_WIDTH") {
            CAM_RES_WIDTH = std::stoi(value);
        } else if (key == "CAM_RES_HEIGHT") {
            CAM_RES_HEIGHT = std::stoi(value);
        } else if (key == "CAM_FPS") {
            CAM_FPS = std::stoi(value);
        } else if (key == "MODEL_INPUT_WIDTH") {
            MODEL_INPUT_WIDTH = std::stoi(value);
        } else if (key == "MODEL_INPUT_HEIGHT") {
            MODEL_INPUT_HEIGHT = std::stoi(value);
        } else if (key == "DISP_WIDTH") {
            DISP_WIDTH = std::stoi(value);
        } else if (key == "DISP_HEIGHT") {
            DISP_HEIGHT = std::stoi(value);
        } else if (key == "VIEW_NUMBER") {
            VIEW_NUMBER = std::stoi(value);
        } else if (key == "MODEL_PATH") {
            MODEL_PATH = value;
        } else if (key == "BACKGROUND_TEXTURE_PATH_JPG") {
            BACKGROUND_TEXTURE_PATH_JPG = value;
        } else if (key == "BACKGROUND_TEXTURE_PATH_BMP") {
            BACKGROUND_TEXTURE_PATH_BMP = value;
        } else if (key == "BACKGROUND_MODEL_PATH") {
            BACKGROUND_MODEL_PATH = value;
        } else if (key == "FOREGROUND_SCALOR") {
            FOREGROUND_SCALOR = parseDoubles(value);
        } 
    }

    std::vector<std::string> split(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::istringstream stream(str);
        std::string token;

        while (std::getline(stream, token, delimiter)) {
            tokens.push_back(token);
        }

        return tokens;
    }

    std::vector<double> parseDoubles(const std::string& str) {
        std::vector<double> values;
        std::istringstream stream(str);
        std::string token;

        while (std::getline(stream, token, ',')) {
            values.push_back(std::stod(token));
        }

        return values;
    }

    std::vector<int> parseInts(const std::string& str) {
        std::vector<int> values;
        std::istringstream stream(str);
        std::string token;
        
        while (std::getline(stream, token, ',')) {
            values.push_back(std::stoi(token));
        }

        return values;
    }
};

                                    
#endif
