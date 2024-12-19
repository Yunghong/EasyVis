// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#pragma once

#include <opencv2/opencv.hpp>
#include "kalmanFilter.h"
#include "kalmanFilterBean.h"
#include "kalmanFilterBean3D.h"
#include "kalmanFilterGrasper.h"
#include "./src/container.hpp"

using namespace cv;
using namespace std;

enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
	STrack(vector<float> tlwh_, float score);
	~STrack();

	vector<float> static tlbr_to_tlwh(vector<float> &tlbr);
	void static multi_predict(vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter);
	void static_tlwh();
	void static_tlbr();
	vector<float> tlwh_to_xyah(vector<float> tlwh_tmp);
	vector<float> to_xyah();
	void mark_lost();
	void mark_removed();
	int next_id(int &count);
	int end_frame();
	
	void activate(int &count, byte_kalman::KalmanFilter &kalman_filter, int frame_id);
	void re_activate(int &count, STrack &new_track, int frame_id, bool new_id = false);
	void update(STrack &new_track, int frame_id);

public:
	bool is_activated;
	int track_id;
	int state;

	vector<float> _tlwh;
	vector<float> tlwh;
	vector<float> tlbr;
	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN mean;
	KAL_COVA covariance;
	float score;

	// to match the object within one image
	// int object_id;

private:
	byte_kalman::KalmanFilter kalman_filter;
};

class STrackBean
{
public:
	STrackBean(vector<float> tlwh_, vector<float> xy_, float score, bean2d orgBean2d);
	~STrackBean();

	vector<float> static tlbr_to_tlwh(vector<float> &tlbr);
	void static multi_predict(vector<STrackBean*> &stracks, byte_kalman_bean::KalmanFilterBean &kalman_filter);
	void static_orgBean();
	void static_xy();
	void static_tlwh();
	void static_tlbr();

	void static_xy_org(float x, float y);
	// void static_tlbr();
	vector<float> tlwh_to_xyah(vector<float> tlwh_tmp);
	vector<float> to_xyah();
	void mark_lost();
	void mark_removed();
	int next_id(int &count);
	int end_frame();
	
	void activate(int &count, byte_kalman_bean::KalmanFilterBean &kalman_filter, int frame_id);
	void re_activate(int &count, STrackBean &new_track, int frame_id, bool new_id = false);
	void update(STrackBean &new_track, int frame_id);

public:
	bool is_activated;
	bool use_prediction;
	int track_id;
	int state;

	vector<float> _tlwh;
	vector<float> tlwh;
	vector<float> tlbr;
	vector<float> _xy;
	vector<float> xy;
	bean2d _orgBean;
	bean2d orgBean;
	// vector<float> tlbr;
	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN_BEAN mean;
	KAL_COVA_BEAN covariance;
	float score;

	// to match the object within one image
	// int object_id;

private:
	byte_kalman_bean::KalmanFilterBean kalman_filter;
};

class STrackGrasper
{
public:
	STrackGrasper(vector<float> tlwh_, vector<float> locator_, vector<float> joint_, vector<float> left_, vector<float> right_, float score, grasper2d orgGrasper2d);
	~STrackGrasper();

	vector<float> static tlbr_to_tlwh(vector<float> &tlbr);
	void static multi_predict(vector<STrackGrasper*> &stracks, byte_kalman_Grasper::KalmanFilterGrasper &kalman_filter);
	void static_orgGrasper();
	void static_locator();
	void static_joint();
	void static_left();
	void static_right();
	void static_tlwh();
	void static_tlbr();

	void static_locator_org(float x, float y);
	void static_joint_org(float x, float y);
	void static_left_org(float x, float y);
	void static_right_org(float x, float y);
	// void static_tlbr();
	vector<float> tlwh_to_xyah(vector<float> tlwh_tmp);
	vector<float> to_xyah();
	void mark_lost();
	void mark_removed();
	int next_id(int &count);
	int end_frame();
	
	void activate(int &count, byte_kalman_Grasper::KalmanFilterGrasper &kalman_filter, int frame_id);
	void re_activate(int &count, STrackGrasper &new_track, int frame_id, bool new_id = false);
	void update(STrackGrasper &new_track, int frame_id);

public:
	bool is_activated;
	int track_id;
	int state;

	vector<float> _tlwh;
	vector<float> tlwh;
	vector<float> tlbr;
	vector<float> _locator;
	vector<float> locator;
	vector<float> _joint;
	vector<float> joint;
	vector<float> _left;
	vector<float> left;
	vector<float> _right;
	vector<float> right;
	grasper2d _orgGrasper;
	grasper2d orgGrasper;
	
	// vector<float> tlbr;
	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN_Grasper mean;
	KAL_COVA_Grasper covariance;
	float score;

	// to match the object within one image
	// int object_id;

private:
	byte_kalman_Grasper::KalmanFilterGrasper kalman_filter;
};

class STrackBean3D
{
public:
	STrackBean3D(vector<float> xyz_, float score);
	~STrackBean3D();

	vector<float> static tlbr_to_tlwh(vector<float> &tlbr);
	void static multi_predict(vector<STrackBean3D*> &stracks, byte_kalman_bean3D::KalmanFilterBean3D &kalman_filter);
	void static_orgBean();
	void static_xyz();
	void static_tlwh();
	void static_tlbr();

	void static_xy_org(float x, float y);
	// void static_tlbr();
	vector<float> tlwh_to_xyah(vector<float> tlwh_tmp);
	vector<float> to_xyah();
	void mark_lost();
	void mark_removed();
	int next_id(int &count);
	int end_frame();
	
	void activate(int &count, byte_kalman_bean3D::KalmanFilterBean3D &kalman_filter, int frame_id);
	void re_activate(int &count, STrackBean3D &new_track, int frame_id, bool new_id = false);
	void update(STrackBean3D &new_track, int frame_id);

public:
	bool is_activated;
	bool use_prediction;
	int track_id;
	int state;

	vector<float> _tlwh;
	vector<float> tlwh;
	vector<float> tlbr;
	vector<float> _xyz;
	vector<float> xyz;
	bean2d _orgBean;
	bean2d orgBean;
	// vector<float> tlbr;
	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN_BEAN3D mean;
	KAL_COVA_BEAN3D covariance;
	float score;

	// to match the object within one image
	// int object_id;

private:
	byte_kalman_bean3D::KalmanFilterBean3D kalman_filter;
};
