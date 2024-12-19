// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#include "kalmanFilterGrasper.h"
#include <Eigen/Cholesky>
#include <iostream>

namespace byte_kalman_Grasper
{
	// const double KalmanFilterGrasper::chi2inv95[10] = {
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
	KalmanFilterGrasper::KalmanFilterGrasper()
	{
		int ndim = 12;
		double dt = 1.;

		_motion_mat = Eigen::MatrixXf::Identity(24, 24);
		for (int i = 0; i < ndim; i++) {
			_motion_mat(i, ndim + i) = dt;
		}
		_update_mat = Eigen::MatrixXf::Identity(12, 24);

		this->_std_weight_position = 1. / 20;
		this->_std_weight_position_locator = 1. / 20;
		this->_std_weight_position_joint = 1. / 20;
		this->_std_weight_position_left = 1. / 20;
		this->_std_weight_position_right = 1. / 20;
		this->_std_weight_velocity = 1. / 160;
		this->_std_weight_velocity_locator = 1. / 160;
		this->_std_weight_velocity_joint = 1. / 160;
		this->_std_weight_velocity_left = 1. / 160;
		this->_std_weight_velocity_right = 1. / 160;
	}

	KAL_DATA_Grasper KalmanFilterGrasper::initiate(const DETECTBOX_Grasper &measurement)
	{
		DETECTBOX_Grasper mean_pos = measurement;
		DETECTBOX_Grasper mean_vel;
		for (int i = 0; i < 12; i++) mean_vel(i) = 0;

		KAL_MEAN_Grasper mean;
		for (int i = 0; i < 24; i++) {
			if (i < 12) {
				mean(i) = mean_pos(i);
				if(mean(i)==0){
					mean(i)+=1e-6;
				}
			}
			else mean(i) = mean_vel(i - 12);
		}

		KAL_MEAN_Grasper std;
		std(0) = 2 * _std_weight_position * measurement[3];
		std(1) = 2 * _std_weight_position * measurement[3];
		std(2) = 1e-2;
		std(3) = 2 * _std_weight_position * measurement[3];
		std(4) = 2 * _std_weight_position_locator * measurement[5];
		std(5) = 2 * _std_weight_position_locator * measurement[5];
		std(6) = 2 * _std_weight_position_joint * measurement[7];
		std(7) = 2 * _std_weight_position_joint * measurement[7];
		std(8) = 2 * _std_weight_position_left * measurement[9];
		std(9) = 2 * _std_weight_position_left * measurement[9];
		std(10) = 2 * _std_weight_position_right * measurement[11];
		std(11) = 2 * _std_weight_position_right * measurement[11];

		std(12) = 10 * _std_weight_velocity * measurement[3];
		std(13) = 10 * _std_weight_velocity * measurement[3];
		std(14) = 1e-5;
		std(15) = 10 * _std_weight_velocity * measurement[3];
		std(16) = 2 * _std_weight_velocity_locator * measurement[5];
		std(17) = 2 * _std_weight_velocity_locator * measurement[5];
		std(18) = 2 * _std_weight_velocity_joint * measurement[7];
		std(19) = 2 * _std_weight_velocity_joint * measurement[7];
		std(20) = 2 * _std_weight_velocity_left * measurement[9];
		std(21) = 2 * _std_weight_velocity_left * measurement[9];
		std(22) = 2 * _std_weight_velocity_right * measurement[11];
		std(23) = 2 * _std_weight_velocity_right * measurement[11];

		KAL_MEAN_Grasper tmp = std.array().square();
		KAL_COVA_Grasper var = tmp.asDiagonal();
		return std::make_pair(mean, var);
	}

	void KalmanFilterGrasper::predict(KAL_MEAN_Grasper &mean, KAL_COVA_Grasper &covariance)
	{
		//revise the data;
		DETECTBOX_Grasper std_pos;
		std_pos << _std_weight_position * mean(3),
			_std_weight_position * mean(3),
			1e-2,
			_std_weight_position * mean(3),
			_std_weight_position_locator*mean(5),
			_std_weight_position_locator*mean(5),
			_std_weight_position_joint*mean(7),
			_std_weight_position_joint*mean(7),
			_std_weight_position_left*mean(9),
			_std_weight_position_left*mean(9),
			_std_weight_position_right*mean(11),
			_std_weight_position_right*mean(11);
		DETECTBOX_Grasper std_vel;
		std_vel << _std_weight_velocity * mean(3),
			_std_weight_velocity * mean(3),
			1e-5,
			_std_weight_velocity * mean(3),
			_std_weight_velocity_locator*mean(5),
			_std_weight_velocity_locator*mean(5),
			_std_weight_velocity_joint*mean(7),
			_std_weight_velocity_joint*mean(7),
			_std_weight_velocity_left*mean(9),
			_std_weight_velocity_left*mean(9),
			_std_weight_velocity_right*mean(11),
			_std_weight_velocity_right*mean(11);


		KAL_MEAN_Grasper tmp;
		tmp.block<1, 12>(0, 0) = std_pos;
		tmp.block<1, 12>(0, 12) = std_vel;
		tmp = tmp.array().square();
		KAL_COVA_Grasper motion_cov = tmp.asDiagonal();
		KAL_MEAN_Grasper mean1 = this->_motion_mat * mean.transpose();
		KAL_COVA_Grasper covariance1 = this->_motion_mat * covariance *(_motion_mat.transpose());
		covariance1 += motion_cov;

		mean = mean1;
		covariance = covariance1;
	}
	// {
	// 	//revise the data;
	// 	DETECTBOX_Grasper std_pos;
	// 	std_pos << _std_weight_position * mean(1),
	// 		_std_weight_position * mean(1);
	// 	DETECTBOX_Grasper std_vel;
	// 	std_vel << _std_weight_velocity * mean(1),
	// 		_std_weight_velocity * mean(1);
	// 	KAL_MEAN_Grasper tmp;
	// 	tmp.block<1, 2>(0, 0) = std_pos;
	// 	tmp.block<1, 2>(0, 2) = std_vel;
	// 	tmp = tmp.array().square();
	// 	KAL_COVA_Grasper motion_cov = tmp.asDiagonal();
	// 	KAL_MEAN_Grasper mean1 = this->_motion_mat * mean.transpose();
	// 	KAL_COVA_Grasper covariance1 = this->_motion_mat * covariance *(_motion_mat.transpose());
	// 	covariance1 += motion_cov;

	// 	mean = mean1;
	// 	covariance = covariance1;
	// }

	KAL_HDATA_Grasper KalmanFilterGrasper::project(const KAL_MEAN_Grasper &mean, const KAL_COVA_Grasper &covariance)
	{
		DETECTBOX_Grasper std;
		// std << _std_weight_position * mean(1), _std_weight_position * mean(1);
		std << 	_std_weight_position * mean(3), 
				_std_weight_position * mean(3),
				1e-1, 
				_std_weight_position * mean(3), 
				_std_weight_position_locator * mean(5), 
				_std_weight_position_locator * mean(5),
				_std_weight_position_joint * mean(7), 
				_std_weight_position_joint * mean(7),
				_std_weight_position_left * mean(9), 
				_std_weight_position_left * mean(9),
				_std_weight_position_right * mean(11), 
				_std_weight_position_right * mean(11);

		KAL_HMEAN_Grasper mean1 = _update_mat * mean.transpose();
		KAL_HCOVA_Grasper covariance1 = _update_mat * covariance * (_update_mat.transpose());
		Eigen::Matrix<float, 12, 12> diag = std.asDiagonal();
		diag = diag.array().square().matrix();
		covariance1 += diag;
		//    covariance1.diagonal() << diag;
		return std::make_pair(mean1, covariance1);
	}

	KAL_DATA_Grasper
		KalmanFilterGrasper::update(
			const KAL_MEAN_Grasper &mean,
			const KAL_COVA_Grasper &covariance,
			const DETECTBOX_Grasper &measurement)
	{
		KAL_HDATA_Grasper pa = project(mean, covariance);
		KAL_HMEAN_Grasper projected_mean = pa.first;
		KAL_HCOVA_Grasper projected_cov = pa.second;

		//chol_factor, lower =
		//scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
		//kalmain_gain =
		//scipy.linalg.cho_solve((cho_factor, lower),
		//np.dot(covariance, self._upadte_mat.T).T,
		//check_finite=False).T
		// std::cout<<"measure"<<std::endl;
		// std::cout<<"mean "<<mean<<std::endl;
		// std::cout<<"covariance "<<covariance<<std::endl;
		// std::cout<<"measure "<<measurement<<std::endl;
		// std::cout<<"projected_mean "<<projected_mean<<std::endl;
		
		Eigen::Matrix<float, 12, 24> B = (covariance * (_update_mat.transpose())).transpose();
		Eigen::Matrix<float, 24, 12> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.8x4
		Eigen::Matrix<float, 1, 12> innovation = measurement - projected_mean; //eg.1x4
		auto tmp = innovation * (kalman_gain.transpose());
		KAL_MEAN_Grasper new_mean = (mean.array() + tmp.array()).matrix();
		KAL_COVA_Grasper new_covariance = covariance - kalman_gain * projected_cov*(kalman_gain.transpose());

		// std::cout<<"new_mean "<<new_mean<<std::endl;

		return std::make_pair(new_mean, new_covariance);
	}

	Eigen::Matrix<float, 1, -1>
		KalmanFilterGrasper::gating_distance(
			const KAL_MEAN_Grasper &mean,
			const KAL_COVA_Grasper &covariance,
			const std::vector<DETECTBOX_Grasper> &measurements,
			bool only_position)
	{
		KAL_HDATA_Grasper pa = this->project(mean, covariance);
		if (only_position) {
			printf("not implement!");
			exit(0);
		}
		KAL_HMEAN_Grasper mean1 = pa.first;
		KAL_HCOVA_Grasper covariance1 = pa.second;

		//    Eigen::Matrix<float, -1, 4, Eigen::RowMajor> d(size, 4);
		DETECTBOXSS_Grasper d(measurements.size(), 12);
		int pos = 0;
		for (DETECTBOX_Grasper box : measurements) {
			d.row(pos++) = box - mean1;
		}
		Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();
		Eigen::Matrix<float, -1, -1> z = factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
		auto zz = ((z.array())*(z.array())).matrix();
		auto square_maha = zz.colwise().sum();
		return square_maha;
	}
}
