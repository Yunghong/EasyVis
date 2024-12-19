// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#include "kalmanFilterBean.h"
#include <Eigen/Cholesky>

namespace byte_kalman_bean
{
	// const double KalmanFilterBean::chi2inv95[10] = {
	// 0,
	// 3.8415,
	// 5.9915,
	// 7.8147,
	// 9.4877,
	// 11.070,
	// 12.592,
	// 14.067,
	// 15.507,
	// 16.919
	// };
	KalmanFilterBean::KalmanFilterBean()
	{
		int ndim = 6;
		double dt = 1.;

		_motion_mat = Eigen::MatrixXf::Identity(12, 12);
		for (int i = 0; i < ndim; i++) {
			_motion_mat(i, ndim + i) = dt;
		}
		_update_mat = Eigen::MatrixXf::Identity(6, 12);

		this->_std_weight_position = 1. / 20;
		this->_std_weight_position_center = 1. / 20;
		this->_std_weight_velocity = 1. / 160;
		this->_std_weight_velocity_center = 1. / 160;
	}

	KAL_DATA_BEAN KalmanFilterBean::initiate(const DETECTBOX_BEAN &measurement)
	{
		DETECTBOX_BEAN mean_pos = measurement;
		DETECTBOX_BEAN mean_vel;
		for (int i = 0; i < 6; i++) mean_vel(i) = 0;

		KAL_MEAN_BEAN mean;
		for (int i = 0; i < 12; i++) {
			if (i < 6) mean(i) = mean_pos(i);
			else mean(i) = mean_vel(i - 6);
		}

		KAL_MEAN_BEAN std;
		std(0) = 2 * _std_weight_position * measurement[3];
		std(1) = 2 * _std_weight_position * measurement[3];
		std(2) = 1e-2;
		std(3) = 2 * _std_weight_position * measurement[3];
		std(4) = 2 * _std_weight_position_center * measurement[5];
		std(5) = 2 * _std_weight_position_center * measurement[5];

		std(6) = 10 * _std_weight_velocity * measurement[3];
		std(7) = 10 * _std_weight_velocity * measurement[3];
		std(8) = 1e-5;
		std(9) = 10 * _std_weight_velocity * measurement[3];
		std(10) = 10 * _std_weight_velocity_center * measurement[5];
		std(11) = 10 * _std_weight_velocity_center * measurement[5];

		KAL_MEAN_BEAN tmp = std.array().square();
		KAL_COVA_BEAN var = tmp.asDiagonal();
		return std::make_pair(mean, var);
	}

	void KalmanFilterBean::predict(KAL_MEAN_BEAN &mean, KAL_COVA_BEAN &covariance)
	{
		//revise the data;
		DETECTBOX_BEAN std_pos;
		std_pos << _std_weight_position * mean(3),
			_std_weight_position * mean(3),
			1e-2,
			_std_weight_position * mean(3),
			_std_weight_position_center*mean(5),
			_std_weight_position_center*mean(5);
		DETECTBOX_BEAN std_vel;
		std_vel << _std_weight_velocity * mean(3),
			_std_weight_velocity * mean(3),
			1e-5,
			_std_weight_velocity * mean(3),
			_std_weight_velocity_center*mean(5),
			_std_weight_velocity_center*mean(5);


		KAL_MEAN_BEAN tmp;
		tmp.block<1, 6>(0, 0) = std_pos;
		tmp.block<1, 6>(0, 6) = std_vel;
		tmp = tmp.array().square();
		KAL_COVA_BEAN motion_cov = tmp.asDiagonal();
		KAL_MEAN_BEAN mean1 = this->_motion_mat * mean.transpose();
		KAL_COVA_BEAN covariance1 = this->_motion_mat * covariance *(_motion_mat.transpose());
		covariance1 += motion_cov;

		mean = mean1;
		covariance = covariance1;
	}
	// {
	// 	//revise the data;
	// 	DETECTBOX_BEAN std_pos;
	// 	std_pos << _std_weight_position * mean(1),
	// 		_std_weight_position * mean(1);
	// 	DETECTBOX_BEAN std_vel;
	// 	std_vel << _std_weight_velocity * mean(1),
	// 		_std_weight_velocity * mean(1);
	// 	KAL_MEAN_BEAN tmp;
	// 	tmp.block<1, 2>(0, 0) = std_pos;
	// 	tmp.block<1, 2>(0, 2) = std_vel;
	// 	tmp = tmp.array().square();
	// 	KAL_COVA_BEAN motion_cov = tmp.asDiagonal();
	// 	KAL_MEAN_BEAN mean1 = this->_motion_mat * mean.transpose();
	// 	KAL_COVA_BEAN covariance1 = this->_motion_mat * covariance *(_motion_mat.transpose());
	// 	covariance1 += motion_cov;

	// 	mean = mean1;
	// 	covariance = covariance1;
	// }

	KAL_HDATA_BEAN KalmanFilterBean::project(const KAL_MEAN_BEAN &mean, const KAL_COVA_BEAN &covariance)
	{
		DETECTBOX_BEAN std;
		// std << _std_weight_position * mean(1), _std_weight_position * mean(1);
		std << _std_weight_position * mean(3), _std_weight_position * mean(3),
			1e-1, _std_weight_position * mean(3), _std_weight_position_center * mean(5), _std_weight_position_center * mean(5);
		KAL_HMEAN_BEAN mean1 = _update_mat * mean.transpose();
		KAL_HCOVA_BEAN covariance1 = _update_mat * covariance * (_update_mat.transpose());
		Eigen::Matrix<float, 6, 6> diag = std.asDiagonal();
		diag = diag.array().square().matrix();
		covariance1 += diag;
		//    covariance1.diagonal() << diag;
		return std::make_pair(mean1, covariance1);
	}

	KAL_DATA_BEAN
		KalmanFilterBean::update(
			const KAL_MEAN_BEAN &mean,
			const KAL_COVA_BEAN &covariance,
			const DETECTBOX_BEAN &measurement)
	{
		KAL_HDATA_BEAN pa = project(mean, covariance);
		KAL_HMEAN_BEAN projected_mean = pa.first;
		KAL_HCOVA_BEAN projected_cov = pa.second;

		//chol_factor, lower =
		//scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
		//kalmain_gain =
		//scipy.linalg.cho_solve((cho_factor, lower),
		//np.dot(covariance, self._upadte_mat.T).T,
		//check_finite=False).T
		Eigen::Matrix<float, 6, 12> B = (covariance * (_update_mat.transpose())).transpose();
		Eigen::Matrix<float, 12, 6> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.8x4
		Eigen::Matrix<float, 1, 6> innovation = measurement - projected_mean; //eg.1x4
		auto tmp = innovation * (kalman_gain.transpose());
		KAL_MEAN_BEAN new_mean = (mean.array() + tmp.array()).matrix();
		KAL_COVA_BEAN new_covariance = covariance - kalman_gain * projected_cov*(kalman_gain.transpose());
		return std::make_pair(new_mean, new_covariance);
	}

	Eigen::Matrix<float, 1, -1>
		KalmanFilterBean::gating_distance(
			const KAL_MEAN_BEAN &mean,
			const KAL_COVA_BEAN &covariance,
			const std::vector<DETECTBOX_BEAN> &measurements,
			bool only_position)
	{
		KAL_HDATA_BEAN pa = this->project(mean, covariance);
		if (only_position) {
			printf("not implement!");
			exit(0);
		}
		KAL_HMEAN_BEAN mean1 = pa.first;
		KAL_HCOVA_BEAN covariance1 = pa.second;

		//    Eigen::Matrix<float, -1, 4, Eigen::RowMajor> d(size, 4);
		DETECTBOXSS_BEAN d(measurements.size(), 6);
		int pos = 0;
		for (DETECTBOX_BEAN box : measurements) {
			d.row(pos++) = box - mean1;
		}
		Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();
		Eigen::Matrix<float, -1, -1> z = factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
		auto zz = ((z.array())*(z.array())).matrix();
		auto square_maha = zz.colwise().sum();
		return square_maha;
	}
}
