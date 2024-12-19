// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git

#pragma once

#include "STrack.h"
#include "./src/container.hpp"

struct BTObject
{
    cv::Rect_<float> rect;
    int label;
    float prob;
	int object_id;
};

struct BTObjectBean
{
    cv::Rect_<float> rect;
	cv::Point2f center;
    int label;
    float prob;
	int object_id;
	bean2d orgBean2d;
};

struct BTObjectGrasper
{
    cv::Rect_<float> rect;
	cv::Point2f locator;
	cv::Point2f joint;
	cv::Point2f left;
	cv::Point2f right;
    int label;
    float prob;
	int object_id;
	grasper2d orgGrasper2d;
};

struct BTObjectBean3D
{
	cv::Point3f center;
    int label;
    float prob;
	int object_id;
};

class BYTETracker
{
public:
	BYTETracker(int frame_rate = 30, int track_buffer = 30);
	~BYTETracker();

	vector<STrack> update(const vector<BTObject>& objects);
	Scalar get_color(int idx);

private:
	vector<STrack*> joint_stracks(vector<STrack*> &tlista, vector<STrack> &tlistb);
	vector<STrack> joint_stracks(vector<STrack> &tlista, vector<STrack> &tlistb);

	vector<STrack> sub_stracks(vector<STrack> &tlista, vector<STrack> &tlistb);
	void remove_duplicate_stracks(vector<STrack> &resa, vector<STrack> &resb, vector<STrack> &stracksa, vector<STrack> &stracksb);

	void linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);
	vector<vector<float> > iou_distance(vector<STrack*> &atracks, vector<STrack> &btracks, int &dist_size, int &dist_size_size);
	vector<vector<float> > iou_distance(vector<STrack> &atracks, vector<STrack> &btracks);
	vector<vector<float> > ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);

	double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

	// count object id
	int count;

private:

	float track_thresh;
	float high_thresh;
	float match_thresh;
	int frame_id;
	int max_time_lost;

	vector<STrack> tracked_stracks;
	vector<STrack> lost_stracks;
	vector<STrack> removed_stracks;
	byte_kalman::KalmanFilter kalman_filter;
};

class BYTETrackerBean
{
public:
	BYTETrackerBean(int frame_rate = 30, int track_buffer = 30);
	~BYTETrackerBean();

	vector<STrackBean> update(const vector<BTObjectBean>& objects);
	Scalar get_color(int idx);

	// count object id
	int count;

private:
	vector<STrackBean*> joint_stracks(vector<STrackBean*> &tlista, vector<STrackBean> &tlistb);
	vector<STrackBean> joint_stracks(vector<STrackBean> &tlista, vector<STrackBean> &tlistb);

	vector<STrackBean> sub_stracks(vector<STrackBean> &tlista, vector<STrackBean> &tlistb);
	void remove_duplicate_stracks(vector<STrackBean> &resa, vector<STrackBean> &resb, vector<STrackBean> &stracksa, vector<STrackBean> &stracksb);

	void linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);
	// vector<vector<float> > iou_distance(vector<STrackBean*> &atracks, vector<STrackBean> &btracks, int &dist_size, int &dist_size_size);
	// vector<vector<float> > iou_distance(vector<STrackBean> &atracks, vector<STrackBean> &btracks);
	vector<vector<float> > avg_distance(vector<STrackBean*> &atracks, vector<STrackBean> &btracks, int &dist_size, int &dist_size_size);
	vector<vector<float> > avg_distance(vector<STrackBean> &atracks, vector<STrackBean> &btracks);
	vector<vector<float> > ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);
	vector<vector<float> > norms(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);
	double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

	

private:

	float track_thresh;
	float high_thresh;
	float match_thresh;
	int frame_id;
	int max_time_lost;

	vector<STrackBean> tracked_stracks;
	vector<STrackBean> lost_stracks;
	vector<STrackBean> removed_stracks;
	byte_kalman_bean::KalmanFilterBean kalman_filter;
};

class BYTETrackerGrasper
{
public:
	BYTETrackerGrasper(int frame_rate = 30, int track_buffer = 30);
	~BYTETrackerGrasper();

	vector<STrackGrasper> update(const vector<BTObjectGrasper>& objects);
	Scalar get_color(int idx);

	int count;

private:
	vector<STrackGrasper*> joint_stracks(vector<STrackGrasper*> &tlista, vector<STrackGrasper> &tlistb);
	vector<STrackGrasper> joint_stracks(vector<STrackGrasper> &tlista, vector<STrackGrasper> &tlistb);

	vector<STrackGrasper> sub_stracks(vector<STrackGrasper> &tlista, vector<STrackGrasper> &tlistb);
	void remove_duplicate_stracks(vector<STrackGrasper> &resa, vector<STrackGrasper> &resb, vector<STrackGrasper> &stracksa, vector<STrackGrasper> &stracksb);

	void linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);
	// vector<vector<float> > iou_distance(vector<STrackGrasper*> &atracks, vector<STrackGrasper> &btracks, int &dist_size, int &dist_size_size);
	// vector<vector<float> > iou_distance(vector<STrackGrasper> &atracks, vector<STrackGrasper> &btracks);
	vector<vector<float> > avg_distance(vector<STrackGrasper*> &atracks, vector<STrackGrasper> &btracks, int &dist_size, int &dist_size_size);
	vector<vector<float> > avg_distance(vector<STrackGrasper> &atracks, vector<STrackGrasper> &btracks);
	vector<vector<float> > ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);
	vector<vector<float> > norms(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);
	double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

	// count object id
	

private:

	float track_thresh;
	float high_thresh;
	float match_thresh;
	int frame_id;
	int max_time_lost;

	vector<STrackGrasper> tracked_stracks;
	vector<STrackGrasper> lost_stracks;
	vector<STrackGrasper> removed_stracks;
	byte_kalman_Grasper::KalmanFilterGrasper kalman_filter;
};

class BYTETrackerBean3D
{
public:
	BYTETrackerBean3D(int frame_rate = 30, int track_buffer = 30);
	~BYTETrackerBean3D();

	vector<STrackBean3D> update(const vector<BTObjectBean3D>& objects);
	Scalar get_color(int idx);

	// count object id
	int count;

private:
	vector<STrackBean3D*> joint_stracks(vector<STrackBean3D*> &tlista, vector<STrackBean3D> &tlistb);
	vector<STrackBean3D> joint_stracks(vector<STrackBean3D> &tlista, vector<STrackBean3D> &tlistb);

	vector<STrackBean3D> sub_stracks(vector<STrackBean3D> &tlista, vector<STrackBean3D> &tlistb);
	void remove_duplicate_stracks(vector<STrackBean3D> &resa, vector<STrackBean3D> &resb, vector<STrackBean3D> &stracksa, vector<STrackBean3D> &stracksb);

	void linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);
	// vector<vector<float> > iou_distance(vector<STrackBean*> &atracks, vector<STrackBean> &btracks, int &dist_size, int &dist_size_size);
	// vector<vector<float> > iou_distance(vector<STrackBean> &atracks, vector<STrackBean> &btracks);
	vector<vector<float> > avg_distance(vector<STrackBean3D*> &atracks, vector<STrackBean3D> &btracks, int &dist_size, int &dist_size_size);
	vector<vector<float> > avg_distance(vector<STrackBean3D> &atracks, vector<STrackBean3D> &btracks);
	vector<vector<float> > ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);
	vector<vector<float> > norms(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);
	double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

	

private:

	float track_thresh;
	float high_thresh;
	float match_thresh;
	int frame_id;
	int max_time_lost;

	vector<STrackBean3D> tracked_stracks;
	vector<STrackBean3D> lost_stracks;
	vector<STrackBean3D> removed_stracks;
	byte_kalman_bean3D::KalmanFilterBean3D kalman_filter;
};
