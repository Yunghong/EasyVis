// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#pragma once

#include "dataTypeGrasper.h"

namespace byte_kalman_Grasper
{
	class KalmanFilterGrasper
	{
	public:
		static const double chi2inv95[10];
		KalmanFilterGrasper();
		KAL_DATA_Grasper initiate(const DETECTBOX_Grasper& measurement);
		void predict(KAL_MEAN_Grasper& mean, KAL_COVA_Grasper& covariance);
		KAL_HDATA_Grasper project(const KAL_MEAN_Grasper& mean, const KAL_COVA_Grasper& covariance);
		KAL_DATA_Grasper update(const KAL_MEAN_Grasper& mean,
			const KAL_COVA_Grasper& covariance,
			const DETECTBOX_Grasper& measurement);

		Eigen::Matrix<float, 1, -1> gating_distance(
			const KAL_MEAN_Grasper& mean,
			const KAL_COVA_Grasper& covariance,
			const std::vector<DETECTBOX_Grasper>& measurements,
			bool only_position = false);

	private:
		Eigen::Matrix<float, 24, 24, Eigen::RowMajor> _motion_mat;
		Eigen::Matrix<float, 12, 24, Eigen::RowMajor> _update_mat;
		float _std_weight_position;
		float _std_weight_velocity;
		float _std_weight_position_locator;
		float _std_weight_velocity_locator;
		float _std_weight_position_joint;
		float _std_weight_velocity_joint;
		float _std_weight_position_left;
		float _std_weight_velocity_left;
		float _std_weight_position_right;
		float _std_weight_velocity_right;
	};
}
