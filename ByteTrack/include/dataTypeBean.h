// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix<float, 1, 6, Eigen::RowMajor> DETECTBOX_BEAN;
typedef Eigen::Matrix<float, -1, 6, Eigen::RowMajor> DETECTBOXSS_BEAN;
typedef Eigen::Matrix<float, 1, 192, Eigen::RowMajor> FEATURE_BEAN;
typedef Eigen::Matrix<float, Eigen::Dynamic, 192, Eigen::RowMajor> FEATURESS_BEAN;
//typedef std::vector<FEATURE> FEATURESS;

//Kalmanfilter
//typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 12, Eigen::RowMajor> KAL_MEAN_BEAN;
typedef Eigen::Matrix<float, 12, 12, Eigen::RowMajor> KAL_COVA_BEAN;
typedef Eigen::Matrix<float, 1, 6, Eigen::RowMajor> KAL_HMEAN_BEAN;
typedef Eigen::Matrix<float, 6, 6, Eigen::RowMajor> KAL_HCOVA_BEAN;
using KAL_DATA_BEAN = std::pair<KAL_MEAN_BEAN, KAL_COVA_BEAN>;
using KAL_HDATA_BEAN = std::pair<KAL_HMEAN_BEAN, KAL_HCOVA_BEAN>;

//main
using RESULT_DATA_BEAN = std::pair<int, DETECTBOX_BEAN>;

//tracker:
using TRACKER_DATA_BEAN = std::pair<int, FEATURESS_BEAN>;
using MATCH_DATA_BEAN = std::pair<int, int>;
// typedef struct t {
// 	std::vector<MATCH_DATA> matches;
// 	std::vector<int> unmatched_tracks;
// 	std::vector<int> unmatched_detections;
// }TRACHER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM_BEAN;
