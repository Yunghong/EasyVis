// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#include "STrack.h"
#include "./src/container.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

STrack::STrack(vector<float> tlwh_, float score)
{
	_tlwh.resize(4);
	_tlwh.assign(tlwh_.begin(), tlwh_.end());

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	tlwh.resize(4);
	tlbr.resize(4);

	static_tlwh();
	static_tlbr();
	frame_id = 0;
	tracklet_len = 0;
	this->score = score;
	start_frame = 0;

	//this->object_id=object_id;
}

STrack::~STrack()
{
}

void STrack::activate(int &count, byte_kalman::KalmanFilter &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id(count);

	vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];
	vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

void STrack::re_activate(int &count, STrack &new_track, int frame_id, bool new_id)
{
	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id = frame_id;
	this->score = new_track.score;
	if (new_id)
		this->track_id = next_id(count);
}

void STrack::update(STrack &new_track, int frame_id)
{
	this->frame_id = frame_id;
	this->tracklet_len++;

	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->state = TrackState::Tracked;
	this->is_activated = true;

	this->score = new_track.score;
}

void STrack::static_tlwh()
{
	if (this->state == TrackState::New)
	{
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
		return;
	}

	tlwh[0] = mean[0];
	tlwh[1] = mean[1];
	tlwh[2] = mean[2];
	tlwh[3] = mean[3];

	tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;
	tlwh[1] -= tlwh[3] / 2;
}

void STrack::static_tlbr()
{
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

vector<float> STrack::tlwh_to_xyah(vector<float> tlwh_tmp)
{
	vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

vector<float> STrack::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

vector<float> STrack::tlbr_to_tlwh(vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void STrack::mark_lost()
{
	state = TrackState::Lost;
}

void STrack::mark_removed()
{
	state = TrackState::Removed;
}

int STrack::next_id(int &count)
{
	count++;
	return count;
}

int STrack::end_frame()
{
	return this->frame_id;
}

void STrack::multi_predict(vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			stracks[i]->mean[7] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
		stracks[i]->static_tlwh();
		stracks[i]->static_tlbr();
	}
}



// ==========================================
// modified Bean

STrackBean::STrackBean(vector<float> tlwh_, vector<float> xy_, float score, bean2d orgBean2d)
{
	_xy.resize(2);
	_xy.assign(xy_.begin(), xy_.end());

	_tlwh.resize(4);
	_tlwh.assign(tlwh_.begin(), tlwh_.end());

	_orgBean=orgBean2d;

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	xy.resize(2);
	tlwh.resize(4);
	tlbr.resize(4);

	static_orgBean();
	static_xy();
	static_tlwh();
	static_tlbr();


	frame_id = 0;
	tracklet_len = 0;
	this->score = score;
	start_frame = 0;

	//this->object_id=object_id;
}

STrackBean::~STrackBean()
{
}

void STrackBean::activate(int &count, byte_kalman_bean::KalmanFilterBean &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id(count);

	vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];

	vector<float> _xy_tmp(2);
	_xy_tmp[0] = this->_xy[0];
	_xy_tmp[1] = this->_xy[1];

	vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
	DETECTBOX_BEAN xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	xyah_box[4] = _xy_tmp[0];
	xyah_box[5] = _xy_tmp[1];
	
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_xy();
	static_tlwh();
	static_tlbr();
	static_xy_org(xyah_box[4],xyah_box[5]);

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

// void STrackBean::activate(int &count, byte_kalman_bean::KalmanFilterBean &kalman_filter, int frame_id)
// {
// 	this->kalman_filter = kalman_filter;
// 	this->track_id = this->next_id(count);

// 	vector<float> _tlwh_tmp(2);
// 	_tlwh_tmp[0] = this->_xy[0];
// 	_tlwh_tmp[1] = this->_xy[1];

// 	// vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
// 	DETECTBOX_BEAN xyah_box;
// 	xyah_box[0] = _tlwh_tmp[0];
// 	xyah_box[1] = _tlwh_tmp[1];

// 	auto mc = this->kalman_filter.initiate(xyah_box);
// 	this->mean = mc.first;
// 	this->covariance = mc.second;

// 	static_xy();
// 	// static_tlbr();

// 	this->tracklet_len = 0;
// 	this->state = TrackState::Tracked;
// 	if (frame_id == 1)
// 	{
// 		this->is_activated = true;
// 	}
// 	//this->is_activated = true;
// 	this->frame_id = frame_id;
// 	this->start_frame = frame_id;
// }

void STrackBean::re_activate(int &count, STrackBean &new_track, int frame_id, bool new_id)
{
	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	// vector<float> xy = new_track.xy;
	DETECTBOX_BEAN xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	xyah_box[4] = new_track.xy[0];
	xyah_box[5] = new_track.xy[1];


	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();
	static_xy();
	static_xy_org(xyah_box[4],xyah_box[5]);

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id = frame_id;
	this->score = new_track.score;
	if (new_id)
		this->track_id = next_id(count);
}

// void STrackBean::re_activate(int &count, STrackBean &new_track, int frame_id, bool new_id)
// {
// 	vector<float> xyah = tlwh_to_xyah(new_track.xy);
// 	DETECTBOX_BEAN xyah_box;
// 	xyah_box[0] = xyah[0];
// 	xyah_box[1] = xyah[1];

// 	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
// 	this->mean = mc.first;
// 	this->covariance = mc.second;

// 	static_xy();
// 	// static_tlbr();

// 	this->tracklet_len = 0;
// 	this->state = TrackState::Tracked;
// 	this->is_activated = true;
// 	this->frame_id = frame_id;
// 	this->score = new_track.score;
// 	if (new_id)
// 		this->track_id = next_id(count);
// }

void STrackBean::update(STrackBean &new_track, int frame_id)
{
	this->frame_id = frame_id;
	this->tracklet_len++;

	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX_BEAN xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	xyah_box[4] = new_track.xy[0];
	xyah_box[5] = new_track.xy[1];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_xy();
	static_tlwh();
	static_tlbr();
	static_xy_org(xyah_box[4],xyah_box[5]);

	this->state = TrackState::Tracked;
	this->is_activated = true;

	this->score = new_track.score;
}

void STrackBean::static_orgBean(){
	orgBean=_orgBean;
}

void STrackBean::static_xy()
{
	if (this->state == TrackState::New)
	{
		xy[0] = _xy[0];
		xy[1] = _xy[1];

		return;
	}

	xy[0] = mean[4];
	xy[1] = mean[5];

	// orgBean.beanCenter=cv::Point2i((int)mean[4],(int)mean[5]);
}

void STrackBean::static_xy_org(float x, float y)
{
	orgBean.beanCenter=cv::Point2i((int)x,(int)y);
}

void STrackBean::static_tlwh()
{
	if (this->state == TrackState::New)
	{
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
		return;
	}

	tlwh[0] = mean[0];
	tlwh[1] = mean[1];
	tlwh[2] = mean[2];
	tlwh[3] = mean[3];

	tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;
	tlwh[1] -= tlwh[3] / 2;
}

void STrackBean::static_tlbr()
{
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

vector<float> STrackBean::tlwh_to_xyah(vector<float> tlwh_tmp)
{
	vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

vector<float> STrackBean::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

vector<float> STrackBean::tlbr_to_tlwh(vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void STrackBean::mark_lost()
{
	state = TrackState::Lost;
}

void STrackBean::mark_removed()
{
	state = TrackState::Removed;
}

int STrackBean::next_id(int &count)
{
	count++;
	return count;
}

int STrackBean::end_frame()
{
	return this->frame_id;
}

void STrackBean::multi_predict(vector<STrackBean*> &stracks, byte_kalman_bean::KalmanFilterBean &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			// stracks[i]->mean[9] = 0;
			stracks[i]->mean[11] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
		stracks[i]->static_xy();
		stracks[i]->static_tlwh();
		stracks[i]->static_tlbr();
	}
}

// ==========================================
// modified Grasper

STrackGrasper::STrackGrasper(vector<float> tlwh_, vector<float> locator_, vector<float> joint_, vector<float> left_, vector<float> right_, float score, grasper2d orgGrasper2d)
{
	_locator.resize(2);
	_locator.assign(locator_.begin(), locator_.end());

	_joint.resize(2);
	_joint.assign(joint_.begin(), joint_.end());

	_left.resize(2);
	_left.assign(left_.begin(), left_.end());

	_right.resize(2);
	_right.assign(right_.begin(), right_.end());

	_tlwh.resize(4);
	_tlwh.assign(tlwh_.begin(), tlwh_.end());

	_orgGrasper=orgGrasper2d;

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	locator.resize(2);
	joint.resize(2);
	left.resize(2);
	right.resize(2);
	tlwh.resize(4);
	tlbr.resize(4);

	static_orgGrasper();
	static_locator();
	static_joint();
	static_left();
	static_right();
	static_tlwh();
	static_tlbr();
	

	frame_id = 0;
	tracklet_len = 0;
	this->score = score;
	start_frame = 0;

	//this->object_id=object_id;
}

STrackGrasper::~STrackGrasper()
{
}

void STrackGrasper::activate(int &count, byte_kalman_Grasper::KalmanFilterGrasper &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id(count);

	vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];

	// vector<float> _xy_tmp(2);
	// _xy_tmp[0] = this->_xy[0];
	// _xy_tmp[1] = this->_xy[1];

	vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
	DETECTBOX_Grasper xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	xyah_box[4] = this->_locator[0];
	xyah_box[5] = this->_locator[1];
	xyah_box[6] = this->_joint[0];
	xyah_box[7] = this->_joint[1];
	xyah_box[8] = this->_left[0];
	xyah_box[9] = this->_left[1];
	xyah_box[10] = this->_right[0];
	xyah_box[11] = this->_right[1];
	
	
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_locator();
	static_joint();
	static_left();
	static_right();
	static_tlwh();
	static_tlbr();

	static_orgGrasper();
	// static_locator_org(xyah_box[4],xyah_box[5]);
	// static_joint_org(xyah_box[6],xyah_box[7]);
	// static_left_org(xyah_box[8],xyah_box[9]);
	// static_right_org(xyah_box[10],xyah_box[11]);

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

// void STrackGrasper::activate(int &count, byte_kalman_Grasper::KalmanFilterGrasper &kalman_filter, int frame_id)
// {
// 	this->kalman_filter = kalman_filter;
// 	this->track_id = this->next_id(count);

// 	vector<float> _tlwh_tmp(2);
// 	_tlwh_tmp[0] = this->_xy[0];
// 	_tlwh_tmp[1] = this->_xy[1];

// 	// vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
// 	DETECTBOX_Grasper xyah_box;
// 	xyah_box[0] = _tlwh_tmp[0];
// 	xyah_box[1] = _tlwh_tmp[1];

// 	auto mc = this->kalman_filter.initiate(xyah_box);
// 	this->mean = mc.first;
// 	this->covariance = mc.second;

// 	static_xy();
// 	// static_tlbr();

// 	this->tracklet_len = 0;
// 	this->state = TrackState::Tracked;
// 	if (frame_id == 1)
// 	{
// 		this->is_activated = true;
// 	}
// 	//this->is_activated = true;
// 	this->frame_id = frame_id;
// 	this->start_frame = frame_id;
// }

void STrackGrasper::re_activate(int &count, STrackGrasper &new_track, int frame_id, bool new_id)
{
	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	// vector<float> xy = new_track.xy;
	DETECTBOX_Grasper xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	xyah_box[4] = new_track.locator[0];
	xyah_box[5] = new_track.locator[1];
	xyah_box[6] = new_track.joint[0];
	xyah_box[7] = new_track.joint[1];
	xyah_box[8] = new_track.left[0];
	xyah_box[9] = new_track.left[1];
	xyah_box[10] = new_track.right[0];
	xyah_box[11] = new_track.right[1];


	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();
	static_locator();
	static_joint();
	static_left();
	static_right();

	_orgGrasper=new_track.orgGrasper;
	static_orgGrasper();

	// static_locator_org(xyah_box[4],xyah_box[5]);
	// static_joint_org(xyah_box[6],xyah_box[7]);
	// static_left_org(xyah_box[8],xyah_box[9]);
	// static_right_org(xyah_box[10],xyah_box[11]);

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id = frame_id;
	this->score = new_track.score;
	if (new_id)
		this->track_id = next_id(count);
}

// void STrackGrasper::re_activate(int &count, STrackGrasper &new_track, int frame_id, bool new_id)
// {
// 	vector<float> xyah = tlwh_to_xyah(new_track.xy);
// 	DETECTBOX_Grasper xyah_box;
// 	xyah_box[0] = xyah[0];
// 	xyah_box[1] = xyah[1];

// 	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
// 	this->mean = mc.first;
// 	this->covariance = mc.second;

// 	static_xy();
// 	// static_tlbr();

// 	this->tracklet_len = 0;
// 	this->state = TrackState::Tracked;
// 	this->is_activated = true;
// 	this->frame_id = frame_id;
// 	this->score = new_track.score;
// 	if (new_id)
// 		this->track_id = next_id(count);
// }

void STrackGrasper::update(STrackGrasper &new_track, int frame_id)
{
	this->frame_id = frame_id;
	this->tracklet_len++;

	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX_Grasper xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	xyah_box[4] = new_track.locator[0];
	xyah_box[5] = new_track.locator[1];
	xyah_box[6] = new_track.joint[0];
	xyah_box[7] = new_track.joint[1];
	xyah_box[8] = new_track.left[0];
	xyah_box[9] = new_track.left[1];
	xyah_box[10] = new_track.right[0];
	xyah_box[11] = new_track.right[1];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_locator();
	static_joint();
	static_left();
	static_right();
	static_tlwh();
	static_tlbr();

	_orgGrasper=new_track.orgGrasper;
	static_orgGrasper();

	// static_locator_org(xyah_box[4],xyah_box[5]);
	// static_joint_org(xyah_box[6],xyah_box[7]);
	// static_left_org(xyah_box[8],xyah_box[9]);
	// static_right_org(xyah_box[10],xyah_box[11]);

	this->state = TrackState::Tracked;
	this->is_activated = true;

	this->score = new_track.score;
}

void STrackGrasper::static_orgGrasper(){
	orgGrasper=_orgGrasper;
}

void STrackGrasper::static_locator()
{
	if (this->state == TrackState::New)
	{
		locator[0] = _locator[0];
		locator[1] = _locator[1];

		return;
	}

	locator[0] = mean[4];
	locator[1] = mean[5];

	// orgGrasper.locator=cv::Point2i((int)mean[4],(int)mean[5]);
}

void STrackGrasper::static_locator_org(float x, float y)
{
	orgGrasper.locator=cv::Point2i((int)x,(int)y);
}

void STrackGrasper::static_joint()
{
	if (this->state == TrackState::New)
	{
		joint[0] = _joint[0];
		joint[1] = _joint[1];

		return;
	}

	joint[0] = mean[6];
	joint[1] = mean[7];

	// orgGrasper.joint=cv::Point2i((int)mean[6],(int)mean[7]);
}

void STrackGrasper::static_joint_org(float x, float y)
{
	orgGrasper.joint=cv::Point2i((int)x,(int)y);
}

void STrackGrasper::static_left()
{
	if (this->state == TrackState::New)
	{
		left[0] = _left[0];
		left[1] = _left[1];

		return;
	}

	left[0] = mean[8];
	left[1] = mean[9];

	// orgGrasper.leftFinger=cv::Point2i((int)mean[8],(int)mean[9]);
}

void STrackGrasper::static_left_org(float x, float y)
{
	orgGrasper.leftFinger=cv::Point2i((int)x,(int)y);
}

void STrackGrasper::static_right()
{
	if (this->state == TrackState::New)
	{
		right[0] = _right[0];
		right[1] = _right[1];

		return;
	}

	right[0] = mean[10];
	right[1] = mean[11];

	// orgGrasper.rightFinger=cv::Point2i((int)mean[10],(int)mean[11]);
}

void STrackGrasper::static_right_org(float x, float y)
{
	orgGrasper.rightFinger=cv::Point2i((int)x,(int)y);
}


void STrackGrasper::static_tlwh()
{
	if (this->state == TrackState::New)
	{
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
		return;
	}

	tlwh[0] = mean[0];
	tlwh[1] = mean[1];
	tlwh[2] = mean[2];
	tlwh[3] = mean[3];

	tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;
	tlwh[1] -= tlwh[3] / 2;
}

void STrackGrasper::static_tlbr()
{
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

vector<float> STrackGrasper::tlwh_to_xyah(vector<float> tlwh_tmp)
{
	vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

vector<float> STrackGrasper::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

vector<float> STrackGrasper::tlbr_to_tlwh(vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void STrackGrasper::mark_lost()
{
	state = TrackState::Lost;
}

void STrackGrasper::mark_removed()
{
	state = TrackState::Removed;
}

int STrackGrasper::next_id(int &count)
{
	count++;
	return count;
}

int STrackGrasper::end_frame()
{
	return this->frame_id;
}

void STrackGrasper::multi_predict(vector<STrackGrasper*> &stracks, byte_kalman_Grasper::KalmanFilterGrasper &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			// stracks[i]->mean[15] = 0;
			// stracks[i]->mean[17] = 0;
			stracks[i]->mean[19] = 0;
			// stracks[i]->mean[21] = 0;
			// stracks[i]->mean[23] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
		stracks[i]->static_locator();
		stracks[i]->static_joint();
		stracks[i]->static_left();
		stracks[i]->static_right();
		stracks[i]->static_tlwh();
		stracks[i]->static_tlbr();
	}
}

// ==========================================
// modified Bean 3D

STrackBean3D::STrackBean3D(vector<float> xyz_, float score)
{
	_xyz.resize(3);
	_xyz.assign(xyz_.begin(), xyz_.end());
	// for(int i=0;i<3;i++){
	// 	std::cout<<_xyz[i]<<" ";
	// }
	// std::cout<<std::endl;

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	xyz.resize(3);

	static_xyz();


	frame_id = 0;
	tracklet_len = 0;
	this->score = score;
	start_frame = 0;

	// this->covariance<<1,0,0,0,0,0,
	// 			0,1,0,0,0,0,
	// 			0,0,1,0,0,0,
	// 			0,0,0,1,0,0,
	// 			0,0,0,0,1,0,
	// 			0,0,0,0,0,1;
	// std::cout<<this->covariance<<std::endl;

	//this->object_id=object_id;
}

STrackBean3D::~STrackBean3D()
{
}

void STrackBean3D::activate(int &count, byte_kalman_bean3D::KalmanFilterBean3D &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id(count);

	// vector<float> _tlwh_tmp(4);
	// _tlwh_tmp[0] = this->_tlwh[0];
	// _tlwh_tmp[1] = this->_tlwh[1];
	// _tlwh_tmp[2] = this->_tlwh[2];
	// _tlwh_tmp[3] = this->_tlwh[3];

	vector<float> _xyz_tmp(3);
	_xyz_tmp[0] = this->_xyz[0];
	_xyz_tmp[1] = this->_xyz[1];
	_xyz_tmp[2] = this->_xyz[2];

	// vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
	DETECTBOX_BEAN3D xyah_box;
	xyah_box[0] = _xyz_tmp[0];
	xyah_box[1] = _xyz_tmp[1];
	xyah_box[2] = _xyz_tmp[2];
	// xyah_box[3] = xyah[3];
	// xyah_box[4] = _xy_tmp[0];

	// std::cout<<"xyah_box"<<std::endl;
	// std::cout<<xyah_box[0]<<" "<<xyah_box[1]<<" "<<xyah_box[2]<<std::endl;
	// xyah_box[5] = _xy_tmp[1];
	
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	// this->covariance<<1e-6,0,0,0,0,0,
	// 					0,1e-6,0,0,0,0,
	// 					0,0,1e-6,0,0,0,
	// 					0,0,0,1e-6,0,0,
	// 					0,0,0,0,1e-6,0,
	// 					0,0,0,0,0,1e-6;

	

	static_xyz();
	// static_tlwh();
	// static_tlbr();
	// static_xy_org(xyah_box[4],xyah_box[5]);

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

void STrackBean3D::re_activate(int &count, STrackBean3D &new_track, int frame_id, bool new_id)
{
	// vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	// vector<float> xy = new_track.xy;
	DETECTBOX_BEAN3D xyah_box;
	xyah_box[0] = new_track.xyz[0];
	xyah_box[1] = new_track.xyz[1];
	xyah_box[2] = new_track.xyz[2];

	// std::cout<<"before"<<std::endl;
	// std::cout<<this->mean<<std::endl;
	
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	// std::cout<<"after"<<std::endl;
	// std::cout<<this->mean<<std::endl;

	// static_tlwh();
	// static_tlbr();
	static_xyz();
	// static_xy_org(xyah_box[4],xyah_box[5]);

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id = frame_id;
	this->score = new_track.score;
	if (new_id)
		this->track_id = next_id(count);
}



void STrackBean3D::update(STrackBean3D &new_track, int frame_id)
{
	this->frame_id = frame_id;
	this->tracklet_len++;

	// vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX_BEAN3D xyah_box;

	xyah_box[0] = new_track.xyz[0];
	xyah_box[1] = new_track.xyz[1];
	xyah_box[2] = new_track.xyz[2];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_xyz();
	// static_tlwh();
	// static_tlbr();
	// static_xy_org(xyah_box[4],xyah_box[5]);

	this->state = TrackState::Tracked;
	this->is_activated = true;

	this->score = new_track.score;
}

// void STrackBean3D::static_orgBean(){
// 	orgBean=_orgBean;
// }

void STrackBean3D::static_xyz()
{
	if (this->state == TrackState::New)
	{
		xyz[0] = _xyz[0];
		xyz[1] = _xyz[1];
		xyz[2] = _xyz[2];

		return;
	}

	xyz[0] = mean[0];
	xyz[1] = mean[1];
	xyz[2] = mean[2];

	// orgBean.beanCenter=cv::Point2i((int)mean[4],(int)mean[5]);
}

// void STrackBean3D::static_xy_org(float x, float y)
// {
// 	orgBean.beanCenter=cv::Point2i((int)x,(int)y);
// }

// void STrackBean3D::static_tlwh()
// {
// 	if (this->state == TrackState::New)
// 	{
// 		tlwh[0] = _tlwh[0];
// 		tlwh[1] = _tlwh[1];
// 		tlwh[2] = _tlwh[2];
// 		tlwh[3] = _tlwh[3];
// 		return;
// 	}

// 	tlwh[0] = mean[0];
// 	tlwh[1] = mean[1];
// 	tlwh[2] = mean[2];
// 	tlwh[3] = mean[3];

// 	tlwh[2] *= tlwh[3];
// 	tlwh[0] -= tlwh[2] / 2;
// 	tlwh[1] -= tlwh[3] / 2;
// }

// void STrackBean3D::static_tlbr()
// {
// 	tlbr.clear();
// 	tlbr.assign(tlwh.begin(), tlwh.end());
// 	tlbr[2] += tlbr[0];
// 	tlbr[3] += tlbr[1];
// }

// vector<float> STrackBean3D::tlwh_to_xyah(vector<float> tlwh_tmp)
// {
// 	vector<float> tlwh_output = tlwh_tmp;
// 	tlwh_output[0] += tlwh_output[2] / 2;
// 	tlwh_output[1] += tlwh_output[3] / 2;
// 	tlwh_output[2] /= tlwh_output[3];
// 	return tlwh_output;
// }

// vector<float> STrackBean3D::to_xyah()
// {
// 	return tlwh_to_xyah(tlwh);
// }

// vector<float> STrackBean3D::tlbr_to_tlwh(vector<float> &tlbr)
// {
// 	tlbr[2] -= tlbr[0];
// 	tlbr[3] -= tlbr[1];
// 	return tlbr;
// }

void STrackBean3D::mark_lost()
{
	state = TrackState::Lost;
}

void STrackBean3D::mark_removed()
{
	state = TrackState::Removed;
}

int STrackBean3D::next_id(int &count)
{
	count++;
	return count;
}

int STrackBean3D::end_frame()
{
	return this->frame_id;
}

void STrackBean3D::multi_predict(vector<STrackBean3D*> &stracks, byte_kalman_bean3D::KalmanFilterBean3D &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			// stracks[i]->mean[9] = 0;
			stracks[i]->mean[5] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
		stracks[i]->static_xyz();
		// stracks[i]->static_tlwh();
		// stracks[i]->static_tlbr();
	}
}
