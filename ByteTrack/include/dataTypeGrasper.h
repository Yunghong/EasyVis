// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix<float, 1, 12, Eigen::RowMajor> DETECTBOX_Grasper;
typedef Eigen::Matrix<float, -1, 12, Eigen::RowMajor> DETECTBOXSS_Grasper;
typedef Eigen::Matrix<float, 1, 384, Eigen::RowMajor> FEATURE_Grasper;
typedef Eigen::Matrix<float, Eigen::Dynamic, 384, Eigen::RowMajor> FEATURESS_Grasper;
//typedef std::vector<FEATURE> FEATURESS;

//Kalmanfilter
//typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 24, Eigen::RowMajor> KAL_MEAN_Grasper;
typedef Eigen::Matrix<float, 24, 24, Eigen::RowMajor> KAL_COVA_Grasper;
typedef Eigen::Matrix<float, 1, 12, Eigen::RowMajor> KAL_HMEAN_Grasper;
typedef Eigen::Matrix<float, 12, 12, Eigen::RowMajor> KAL_HCOVA_Grasper;
using KAL_DATA_Grasper = std::pair<KAL_MEAN_Grasper, KAL_COVA_Grasper>;
using KAL_HDATA_Grasper = std::pair<KAL_HMEAN_Grasper, KAL_HCOVA_Grasper>;

//main
using RESULT_DATA_Grasper = std::pair<int, DETECTBOX_Grasper>;

//tracker:
using TRACKER_DATA_Grasper = std::pair<int, FEATURESS_Grasper>;
using MATCH_DATA_Grasper = std::pair<int, int>;
// typedef struct t {
// 	std::vector<MATCH_DATA> matches;
// 	std::vector<int> unmatched_tracks;
// 	std::vector<int> unmatched_detections;
// }TRACHER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM_Grasper;
