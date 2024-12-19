// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#pragma once

#include "dataTypeBean.h"

namespace byte_kalman_bean
{
	class KalmanFilterBean
	{
	public:
		static const double chi2inv95[10];
		KalmanFilterBean();
		KAL_DATA_BEAN initiate(const DETECTBOX_BEAN& measurement);
		void predict(KAL_MEAN_BEAN& mean, KAL_COVA_BEAN& covariance);
		KAL_HDATA_BEAN project(const KAL_MEAN_BEAN& mean, const KAL_COVA_BEAN& covariance);
		KAL_DATA_BEAN update(const KAL_MEAN_BEAN& mean,
			const KAL_COVA_BEAN& covariance,
			const DETECTBOX_BEAN& measurement);

		Eigen::Matrix<float, 1, -1> gating_distance(
			const KAL_MEAN_BEAN& mean,
			const KAL_COVA_BEAN& covariance,
			const std::vector<DETECTBOX_BEAN>& measurements,
			bool only_position = false);

	private:
		Eigen::Matrix<float, 12, 12, Eigen::RowMajor> _motion_mat;
		Eigen::Matrix<float, 6, 12, Eigen::RowMajor> _update_mat;
		float _std_weight_position;
		float _std_weight_velocity;
		float _std_weight_position_center;
		float _std_weight_velocity_center;
	};
}
