// cite https://github.com/Vertical-Beach/ByteTrack-cpp.git
// revised and extended

#include "BYTETracker.h"
#include "lapjv.h"

vector<STrack*> BYTETracker::joint_stracks(vector<STrack*> &tlista, vector<STrack> &tlistb)
{
	map<int, int> exists;
	vector<STrack*> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i]->track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(&tlistb[i]);
		}
	}
	return res;
}

vector<STrack> BYTETracker::joint_stracks(vector<STrack> &tlista, vector<STrack> &tlistb)
{
	map<int, int> exists;
	vector<STrack> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i].track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(tlistb[i]);
		}
	}
	return res;
}

vector<STrack> BYTETracker::sub_stracks(vector<STrack> &tlista, vector<STrack> &tlistb)
{
	map<int, STrack> stracks;
	for (int i = 0; i < tlista.size(); i++)
	{
		stracks.insert(pair<int, STrack>(tlista[i].track_id, tlista[i]));
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (stracks.count(tid) != 0)
		{
			stracks.erase(tid);
		}
	}

	vector<STrack> res;
	std::map<int, STrack>::iterator  it;
	for (it = stracks.begin(); it != stracks.end(); ++it)
	{
		res.push_back(it->second);
	}

	return res;
}

void BYTETracker::remove_duplicate_stracks(vector<STrack> &resa, vector<STrack> &resb, vector<STrack> &stracksa, vector<STrack> &stracksb)
{
	vector<vector<float> > pdist = iou_distance(stracksa, stracksb);
	vector<pair<int, int> > pairs;
	for (int i = 0; i < pdist.size(); i++)
	{
		for (int j = 0; j < pdist[i].size(); j++)
		{
			if (pdist[i][j] < 0.15)
			{
				pairs.push_back(pair<int, int>(i, j));
			}
		}
	}

	vector<int> dupa, dupb;
	for (int i = 0; i < pairs.size(); i++)
	{
		int timep = stracksa[pairs[i].first].frame_id - stracksa[pairs[i].first].start_frame;
		int timeq = stracksb[pairs[i].second].frame_id - stracksb[pairs[i].second].start_frame;
		if (timep > timeq)
			dupb.push_back(pairs[i].second);
		else
			dupa.push_back(pairs[i].first);
	}

	for (int i = 0; i < stracksa.size(); i++)
	{
		vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
		if (iter == dupa.end())
		{
			resa.push_back(stracksa[i]);
		}
	}

	for (int i = 0; i < stracksb.size(); i++)
	{
		vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
		if (iter == dupb.end())
		{
			resb.push_back(stracksb[i]);
		}
	}
}

void BYTETracker::linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
	vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b)
{
	if (cost_matrix.size() == 0)
	{
		for (int i = 0; i < cost_matrix_size; i++)
		{
			unmatched_a.push_back(i);
		}
		for (int i = 0; i < cost_matrix_size_size; i++)
		{
			unmatched_b.push_back(i);
		}
		return;
	}

	vector<int> rowsol; vector<int> colsol;
	float c = lapjv(cost_matrix, rowsol, colsol, true, thresh);
	for (int i = 0; i < rowsol.size(); i++)
	{
		if (rowsol[i] >= 0)
		{
			vector<int> match;
			match.push_back(i);
			match.push_back(rowsol[i]);
			matches.push_back(match);
		}
		else
		{
			unmatched_a.push_back(i);
		}
	}

	for (int i = 0; i < colsol.size(); i++)
	{
		if (colsol[i] < 0)
		{
			unmatched_b.push_back(i);
		}
	}
}

vector<vector<float> > BYTETracker::ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs)
{
	vector<vector<float> > ious;
	if (atlbrs.size()*btlbrs.size() == 0)
		return ious;

	ious.resize(atlbrs.size());
	for (int i = 0; i < ious.size(); i++)
	{
		ious[i].resize(btlbrs.size());
	}

	//bbox_ious
	for (int k = 0; k < btlbrs.size(); k++)
	{
		vector<float> ious_tmp;
		float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
		for (int n = 0; n < atlbrs.size(); n++)
		{
			float iw = min(atlbrs[n][2], btlbrs[k][2]) - max(atlbrs[n][0], btlbrs[k][0]) + 1;
			if (iw > 0)
			{
				float ih = min(atlbrs[n][3], btlbrs[k][3]) - max(atlbrs[n][1], btlbrs[k][1]) + 1;
				if(ih > 0)
				{
					float ua = (atlbrs[n][2] - atlbrs[n][0] + 1)*(atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
					ious[n][k] = iw * ih / ua;
				}
				else
				{
					ious[n][k] = 0.0;
				}
			}
			else
			{
				ious[n][k] = 0.0;
			}
		}
	}

	return ious;
}

vector<vector<float> > BYTETracker::iou_distance(vector<STrack*> &atracks, vector<STrack> &btracks, int &dist_size, int &dist_size_size)
{
	vector<vector<float> > cost_matrix;
	if (atracks.size() * btracks.size() == 0)
	{
		dist_size = atracks.size();
		dist_size_size = btracks.size();
		return cost_matrix;
	}
	vector<vector<float> > atlbrs, btlbrs;
	for (int i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i]->tlbr);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
	}

	dist_size = atracks.size();
	dist_size_size = btracks.size();

	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	
	for (int i = 0; i < _ious.size();i++)
	{
		vector<float> _iou;
		for (int j = 0; j < _ious[i].size(); j++)
		{
			_iou.push_back(1 - _ious[i][j]);
		}
		cost_matrix.push_back(_iou);
	}

	return cost_matrix;
}

vector<vector<float> > BYTETracker::iou_distance(vector<STrack> &atracks, vector<STrack> &btracks)
{
	vector<vector<float> > atlbrs, btlbrs;
	for (int i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i].tlbr);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
	}

	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	vector<vector<float> > cost_matrix;
	for (int i = 0; i < _ious.size(); i++)
	{
		vector<float> _iou;
		for (int j = 0; j < _ious[i].size(); j++)
		{
			_iou.push_back(1 - _ious[i][j]);
		}
		cost_matrix.push_back(_iou);
	}

	return cost_matrix;
}

double BYTETracker::lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
	bool extend_cost, float cost_limit, bool return_cost)
{
	vector<vector<float> > cost_c;
	cost_c.assign(cost.begin(), cost.end());

	vector<vector<float> > cost_c_extended;

	int n_rows = cost.size();
	int n_cols = cost[0].size();
	rowsol.resize(n_rows);
	colsol.resize(n_cols);

	int n = 0;
	if (n_rows == n_cols)
	{
		n = n_rows;
	}
	else
	{
		if (!extend_cost)
		{
			cout << "set extend_cost=True" << endl;
			system("pause");
			exit(0);
		}
	}
		
	if (extend_cost || cost_limit < LONG_MAX)
	{
		n = n_rows + n_cols;
		cost_c_extended.resize(n);
		for (int i = 0; i < cost_c_extended.size(); i++)
			cost_c_extended[i].resize(n);

		if (cost_limit < LONG_MAX)
		{
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_limit / 2.0;
				}
			}
		}
		else
		{
			float cost_max = -1;
			for (int i = 0; i < cost_c.size(); i++)
			{
				for (int j = 0; j < cost_c[i].size(); j++)
				{
					if (cost_c[i][j] > cost_max)
						cost_max = cost_c[i][j];
				}
			}
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_max + 1;
				}
			}
		}

		for (int i = n_rows; i < cost_c_extended.size(); i++)
		{
			for (int j = n_cols; j < cost_c_extended[i].size(); j++)
			{
				cost_c_extended[i][j] = 0;
			}
		}
		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				cost_c_extended[i][j] = cost_c[i][j];
			}
		}

		cost_c.clear();
		cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
	}

	double **cost_ptr;
	cost_ptr = new double *[sizeof(double *) * n];
	for (int i = 0; i < n; i++)
		cost_ptr[i] = new double[sizeof(double) * n];

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cost_ptr[i][j] = cost_c[i][j];
		}
	}

	int* x_c = new int[sizeof(int) * n];
	int *y_c = new int[sizeof(int) * n];

	int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
	if (ret != 0)
	{
		cout << "Calculate Wrong!" << endl;
		system("pause");
		exit(0);
	}

	double opt = 0.0;

	if (n != n_rows)
	{
		for (int i = 0; i < n; i++)
		{
			if (x_c[i] >= n_cols)
				x_c[i] = -1;
			if (y_c[i] >= n_rows)
				y_c[i] = -1;
		}
		for (int i = 0; i < n_rows; i++)
		{
			rowsol[i] = x_c[i];
		}
		for (int i = 0; i < n_cols; i++)
		{
			colsol[i] = y_c[i];
		}

		if (return_cost)
		{
			for (int i = 0; i < rowsol.size(); i++)
			{
				if (rowsol[i] != -1)
				{
					//cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
					opt += cost_ptr[i][rowsol[i]];
				}
			}
		}
	}
	else if (return_cost)
	{
		for (int i = 0; i < rowsol.size(); i++)
		{
			opt += cost_ptr[i][rowsol[i]];
		}
	}

	for (int i = 0; i < n; i++)
	{
		delete[]cost_ptr[i];
	}
	delete[]cost_ptr;
	delete[]x_c;
	delete[]y_c;

	return opt;
}

Scalar BYTETracker::get_color(int idx)
{
	idx += 3;
	return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}

// ====================================
// modified Bean

vector<STrackBean*> BYTETrackerBean::joint_stracks(vector<STrackBean*> &tlista, vector<STrackBean> &tlistb)
{
	map<int, int> exists;
	vector<STrackBean*> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i]->track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(&tlistb[i]);
		}
	}
	return res;
}

vector<STrackBean> BYTETrackerBean::joint_stracks(vector<STrackBean> &tlista, vector<STrackBean> &tlistb)
{
	map<int, int> exists;
	vector<STrackBean> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i].track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(tlistb[i]);
		}
	}
	return res;
}

vector<STrackBean> BYTETrackerBean::sub_stracks(vector<STrackBean> &tlista, vector<STrackBean> &tlistb)
{
	map<int, STrackBean> stracks;
	for (int i = 0; i < tlista.size(); i++)
	{
		stracks.insert(pair<int, STrackBean>(tlista[i].track_id, tlista[i]));
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (stracks.count(tid) != 0)
		{
			stracks.erase(tid);
		}
	}

	vector<STrackBean> res;
	std::map<int, STrackBean>::iterator  it;
	for (it = stracks.begin(); it != stracks.end(); ++it)
	{
		res.push_back(it->second);
	}

	return res;
}

void BYTETrackerBean::remove_duplicate_stracks(vector<STrackBean> &resa, vector<STrackBean> &resb, vector<STrackBean> &stracksa, vector<STrackBean> &stracksb)
{
	vector<vector<float> > pdist = avg_distance(stracksa, stracksb);
	vector<pair<int, int> > pairs;
	for (int i = 0; i < pdist.size(); i++)
	{
		for (int j = 0; j < pdist[i].size(); j++)
		{
			if (pdist[i][j] < 0.15)
			{
				pairs.push_back(pair<int, int>(i, j));
			}
		}
	}

	vector<int> dupa, dupb;
	for (int i = 0; i < pairs.size(); i++)
	{
		int timep = stracksa[pairs[i].first].frame_id - stracksa[pairs[i].first].start_frame;
		int timeq = stracksb[pairs[i].second].frame_id - stracksb[pairs[i].second].start_frame;
		if (timep > timeq)
			dupb.push_back(pairs[i].second);
		else
			dupa.push_back(pairs[i].first);
	}

	for (int i = 0; i < stracksa.size(); i++)
	{
		vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
		if (iter == dupa.end())
		{
			resa.push_back(stracksa[i]);
		}
	}

	for (int i = 0; i < stracksb.size(); i++)
	{
		vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
		if (iter == dupb.end())
		{
			resb.push_back(stracksb[i]);
		}
	}
}

void BYTETrackerBean::linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
	vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b)
{
	if (cost_matrix.size() == 0)
	{
		for (int i = 0; i < cost_matrix_size; i++)
		{
			unmatched_a.push_back(i);
		}
		for (int i = 0; i < cost_matrix_size_size; i++)
		{
			unmatched_b.push_back(i);
		}
		return;
	}

	vector<int> rowsol; vector<int> colsol;
	float c = lapjv(cost_matrix, rowsol, colsol, true, thresh);
	for (int i = 0; i < rowsol.size(); i++)
	{
		if (rowsol[i] >= 0)
		{
			vector<int> match;
			match.push_back(i);
			match.push_back(rowsol[i]);
			matches.push_back(match);
		}
		else
		{
			unmatched_a.push_back(i);
		}
	}

	for (int i = 0; i < colsol.size(); i++)
	{
		if (colsol[i] < 0)
		{
			unmatched_b.push_back(i);
		}
	}
}

vector<vector<float> > BYTETrackerBean::norms(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs)
{
	vector<vector<float> > distances;
	if (atlbrs.size()*btlbrs.size() == 0)
		return distances;

	distances.resize(atlbrs.size());
	for (int i = 0; i < distances.size(); i++)
	{
		distances[i].resize(btlbrs.size());
	}

	//distances
	for (int k = 0; k < btlbrs.size(); k++)
	{
		//vector<float> ious_tmp;
		//float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
		for (int n = 0; n < atlbrs.size(); n++)
		{
			distances[n][k]=(640-sqrt((btlbrs[k][0]-atlbrs[n][0])*(btlbrs[k][0]-atlbrs[n][0])
						       +(btlbrs[k][1]-atlbrs[n][1])*(btlbrs[k][1]-atlbrs[n][1])))/640;
			// float iw = min(atlbrs[n][2], btlbrs[k][2]) - max(atlbrs[n][0], btlbrs[k][0]) + 1;
			// if (iw > 0)
			// {
			// 	float ih = min(atlbrs[n][3], btlbrs[k][3]) - max(atlbrs[n][1], btlbrs[k][1]) + 1;
			// 	if(ih > 0)
			// 	{
			// 		float ua = (atlbrs[n][2] - atlbrs[n][0] + 1)*(atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
			// 		ious[n][k] = iw * ih / ua;
			// 	}
			// 	else
			// 	{
			// 		ious[n][k] = 0.0;
			// 	}
			// }
			// else
			// {
			// 	ious[n][k] = 0.0;
			// }
		}
	}

	return distances;
}

vector<vector<float> > BYTETrackerBean::ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs)
{
	vector<vector<float> > ious;
	if (atlbrs.size()*btlbrs.size() == 0)
		return ious;

	ious.resize(atlbrs.size());
	for (int i = 0; i < ious.size(); i++)
	{
		ious[i].resize(btlbrs.size());
	}

	//bbox_ious
	for (int k = 0; k < btlbrs.size(); k++)
	{
		vector<float> ious_tmp;
		float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
		for (int n = 0; n < atlbrs.size(); n++)
		{
			float iw = min(atlbrs[n][2], btlbrs[k][2]) - max(atlbrs[n][0], btlbrs[k][0]) + 1;
			if (iw > 0)
			{
				float ih = min(atlbrs[n][3], btlbrs[k][3]) - max(atlbrs[n][1], btlbrs[k][1]) + 1;
				if(ih > 0)
				{
					float ua = (atlbrs[n][2] - atlbrs[n][0] + 1)*(atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
					ious[n][k] = iw * ih / ua;
				}
				else
				{
					ious[n][k] = 0.0;
				}
			}
			else
			{
				ious[n][k] = 0.0;
			}
		}
	}

	return ious;
}

// vector<vector<float> > BYTETrackerBean::norm_distance(vector<STrackBean*> &atracks, vector<STrackBean> &btracks, int &dist_size, int &dist_size_size)
// {
// 	vector<vector<float> > cost_matrix;
// 	if (atracks.size() * btracks.size() == 0)
// 	{
// 		dist_size = atracks.size();
// 		dist_size_size = btracks.size();
// 		return cost_matrix;
// 	}
// 	vector<vector<float> > atlbrs, btlbrs;
// 	for (int i = 0; i < atracks.size(); i++)
// 	{
// 		atlbrs.push_back(atracks[i]->xy);
// 	}
// 	for (int i = 0; i < btracks.size(); i++)
// 	{
// 		btlbrs.push_back(btracks[i].xy);
// 	}

// 	dist_size = atracks.size();
// 	dist_size_size = btracks.size();

// 	vector<vector<float> > _distances = norms(atlbrs, btlbrs);
	
// 	for (int i = 0; i < _distances.size();i++)
// 	{
// 		vector<float> _iou;
// 		for (int j = 0; j < _distances[i].size(); j++)
// 		{
// 			_iou.push_back(1 - _distances[i][j]);
// 		}
// 		cost_matrix.push_back(_iou);
// 	}

// 	return cost_matrix;
// }

// vector<vector<float> > BYTETrackerBean::iou_distance(vector<STrackBean*> &atracks, vector<STrackBean> &btracks, int &dist_size, int &dist_size_size)
// {
// 	vector<vector<float> > cost_matrix;
// 	if (atracks.size() * btracks.size() == 0)
// 	{
// 		dist_size = atracks.size();
// 		dist_size_size = btracks.size();
// 		return cost_matrix;
// 	}
// 	vector<vector<float> > atlbrs, btlbrs;
// 	for (int i = 0; i < atracks.size(); i++)
// 	{
// 		atlbrs.push_back(atracks[i]->tlbr);
// 	}
// 	for (int i = 0; i < btracks.size(); i++)
// 	{
// 		btlbrs.push_back(btracks[i].tlbr);
// 	}

// 	dist_size = atracks.size();
// 	dist_size_size = btracks.size();

// 	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	
// 	for (int i = 0; i < _ious.size();i++)
// 	{
// 		vector<float> _iou;
// 		for (int j = 0; j < _ious[i].size(); j++)
// 		{
// 			_iou.push_back(1 - _ious[i][j]);
// 		}
// 		cost_matrix.push_back(_iou);
// 	}

// 	return cost_matrix;
// }

vector<vector<float> > BYTETrackerBean::avg_distance(vector<STrackBean*> &atracks, vector<STrackBean> &btracks, int &dist_size, int &dist_size_size)
{
	vector<vector<float> > cost_matrix;
	if (atracks.size() * btracks.size() == 0)
	{
		dist_size = atracks.size();
		dist_size_size = btracks.size();
		return cost_matrix;
	}
	vector<vector<float> > atlbrs, btlbrs;
	vector<vector<float> > axys, bxys;
	for (int i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i]->tlbr);
		axys.push_back(atracks[i]->xy);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
		bxys.push_back(btracks[i].xy);
	}

	dist_size = atracks.size();
	dist_size_size = btracks.size();

	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	vector<vector<float> > _distances = norms(axys, bxys);
	
	for (int i = 0; i < _ious.size();i++)
	{
		vector<float> _avg;
		for (int j = 0; j < _ious[i].size(); j++)
		{
			_avg.push_back(1 - (_ious[i][j]+_distances[i][j])/2);
			// _avg.push_back(1 - _distances[i][j]);
		}
		cost_matrix.push_back(_avg);
	}

	return cost_matrix;
}

// vector<vector<float> > BYTETrackerBean::norm_distance(vector<STrackBean> &atracks, vector<STrackBean> &btracks)
// {
// 	vector<vector<float> > atlbrs, btlbrs;
// 	for (int i = 0; i < atracks.size(); i++)
// 	{
// 		atlbrs.push_back(atracks[i].xy);
// 	}
// 	for (int i = 0; i < btracks.size(); i++)
// 	{
// 		btlbrs.push_back(btracks[i].xy);
// 	}

// 	vector<vector<float> > _distances = norms(atlbrs, btlbrs);
// 	vector<vector<float> > cost_matrix;
// 	for (int i = 0; i < _distances.size(); i++)
// 	{
// 		vector<float> _iou;
// 		for (int j = 0; j < _distances[i].size(); j++)
// 		{
// 			_iou.push_back(1 - _distances[i][j]);
// 		}
// 		cost_matrix.push_back(_iou);
// 	}

// 	return cost_matrix;
// }

vector<vector<float> > BYTETrackerBean::avg_distance(vector<STrackBean> &atracks, vector<STrackBean> &btracks)
{
	vector<vector<float> > atlbrs, btlbrs;
	vector<vector<float> > axys, bxys;
	for (int i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i].tlbr);
		axys.push_back(atracks[i].xy);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
		bxys.push_back(btracks[i].xy);
	}

	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	vector<vector<float> > _distances = norms(axys, bxys);
	vector<vector<float> > cost_matrix;
	for (int i = 0; i < _ious.size(); i++)
	{
		vector<float> _avg;
		for (int j = 0; j < _ious[i].size(); j++)
		{
			_avg.push_back(1 - (_ious[i][j]+_distances[i][j])/2);
			// _avg.push_back(1 - _distances[i][j]);
		}
		cost_matrix.push_back(_avg);
	}

	return cost_matrix;
}

// vector<vector<float> > BYTETrackerBean::iou_distance(vector<STrackBean> &atracks, vector<STrackBean> &btracks)
// {
// 	vector<vector<float> > atlbrs, btlbrs;
// 	for (int i = 0; i < atracks.size(); i++)
// 	{
// 		atlbrs.push_back(atracks[i].tlbr);
// 	}
// 	for (int i = 0; i < btracks.size(); i++)
// 	{
// 		btlbrs.push_back(btracks[i].tlbr);
// 	}

// 	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
// 	vector<vector<float> > cost_matrix;
// 	for (int i = 0; i < _ious.size(); i++)
// 	{
// 		vector<float> _iou;
// 		for (int j = 0; j < _ious[i].size(); j++)
// 		{
// 			_iou.push_back(1 - _ious[i][j]);
// 		}
// 		cost_matrix.push_back(_iou);
// 	}

// 	return cost_matrix;
// }

double BYTETrackerBean::lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
	bool extend_cost, float cost_limit, bool return_cost)
{
	vector<vector<float> > cost_c;
	cost_c.assign(cost.begin(), cost.end());

	vector<vector<float> > cost_c_extended;

	int n_rows = cost.size();
	int n_cols = cost[0].size();
	rowsol.resize(n_rows);
	colsol.resize(n_cols);

	int n = 0;
	if (n_rows == n_cols)
	{
		n = n_rows;
	}
	else
	{
		if (!extend_cost)
		{
			cout << "set extend_cost=True" << endl;
			system("pause");
			exit(0);
		}
	}
		
	if (extend_cost || cost_limit < LONG_MAX)
	{
		n = n_rows + n_cols;
		cost_c_extended.resize(n);
		for (int i = 0; i < cost_c_extended.size(); i++)
			cost_c_extended[i].resize(n);

		if (cost_limit < LONG_MAX)
		{
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_limit / 2.0;
				}
			}
		}
		else
		{
			float cost_max = -1;
			for (int i = 0; i < cost_c.size(); i++)
			{
				for (int j = 0; j < cost_c[i].size(); j++)
				{
					if (cost_c[i][j] > cost_max)
						cost_max = cost_c[i][j];
				}
			}
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_max + 1;
				}
			}
		}

		for (int i = n_rows; i < cost_c_extended.size(); i++)
		{
			for (int j = n_cols; j < cost_c_extended[i].size(); j++)
			{
				cost_c_extended[i][j] = 0;
			}
		}
		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				cost_c_extended[i][j] = cost_c[i][j];
			}
		}

		cost_c.clear();
		cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
	}

	double **cost_ptr;
	cost_ptr = new double *[sizeof(double *) * n];
	for (int i = 0; i < n; i++)
		cost_ptr[i] = new double[sizeof(double) * n];

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cost_ptr[i][j] = cost_c[i][j];
		}
	}

	int* x_c = new int[sizeof(int) * n];
	int *y_c = new int[sizeof(int) * n];

	int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
	if (ret != 0)
	{
		cout << "Calculate Wrong!" << endl;
		system("pause");
		exit(0);
	}

	double opt = 0.0;

	if (n != n_rows)
	{
		for (int i = 0; i < n; i++)
		{
			if (x_c[i] >= n_cols)
				x_c[i] = -1;
			if (y_c[i] >= n_rows)
				y_c[i] = -1;
		}
		for (int i = 0; i < n_rows; i++)
		{
			rowsol[i] = x_c[i];
		}
		for (int i = 0; i < n_cols; i++)
		{
			colsol[i] = y_c[i];
		}

		if (return_cost)
		{
			for (int i = 0; i < rowsol.size(); i++)
			{
				if (rowsol[i] != -1)
				{
					//cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
					opt += cost_ptr[i][rowsol[i]];
				}
			}
		}
	}
	else if (return_cost)
	{
		for (int i = 0; i < rowsol.size(); i++)
		{
			opt += cost_ptr[i][rowsol[i]];
		}
	}

	for (int i = 0; i < n; i++)
	{
		delete[]cost_ptr[i];
	}
	delete[]cost_ptr;
	delete[]x_c;
	delete[]y_c;

	return opt;
}

Scalar BYTETrackerBean::get_color(int idx)
{
	idx += 3;
	return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}

// ====================================
// modified Grasper

vector<STrackGrasper*> BYTETrackerGrasper::joint_stracks(vector<STrackGrasper*> &tlista, vector<STrackGrasper> &tlistb)
{
	map<int, int> exists;
	vector<STrackGrasper*> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i]->track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(&tlistb[i]);
		}
	}
	return res;
}

vector<STrackGrasper> BYTETrackerGrasper::joint_stracks(vector<STrackGrasper> &tlista, vector<STrackGrasper> &tlistb)
{
	map<int, int> exists;
	vector<STrackGrasper> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i].track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(tlistb[i]);
		}
	}
	return res;
}

vector<STrackGrasper> BYTETrackerGrasper::sub_stracks(vector<STrackGrasper> &tlista, vector<STrackGrasper> &tlistb)
{
	map<int, STrackGrasper> stracks;
	for (int i = 0; i < tlista.size(); i++)
	{
		stracks.insert(pair<int, STrackGrasper>(tlista[i].track_id, tlista[i]));
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (stracks.count(tid) != 0)
		{
			stracks.erase(tid);
		}
	}

	vector<STrackGrasper> res;
	std::map<int, STrackGrasper>::iterator  it;
	for (it = stracks.begin(); it != stracks.end(); ++it)
	{
		res.push_back(it->second);
	}

	return res;
}

void BYTETrackerGrasper::remove_duplicate_stracks(vector<STrackGrasper> &resa, vector<STrackGrasper> &resb, vector<STrackGrasper> &stracksa, vector<STrackGrasper> &stracksb)
{
	vector<vector<float> > pdist = avg_distance(stracksa, stracksb);
	vector<pair<int, int> > pairs;
	for (int i = 0; i < pdist.size(); i++)
	{
		for (int j = 0; j < pdist[i].size(); j++)
		{
			if (pdist[i][j] < 0.15)
			{
				pairs.push_back(pair<int, int>(i, j));
			}
		}
	}

	vector<int> dupa, dupb;
	for (int i = 0; i < pairs.size(); i++)
	{
		int timep = stracksa[pairs[i].first].frame_id - stracksa[pairs[i].first].start_frame;
		int timeq = stracksb[pairs[i].second].frame_id - stracksb[pairs[i].second].start_frame;
		if (timep > timeq)
			dupb.push_back(pairs[i].second);
		else
			dupa.push_back(pairs[i].first);
	}

	for (int i = 0; i < stracksa.size(); i++)
	{
		vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
		if (iter == dupa.end())
		{
			resa.push_back(stracksa[i]);
		}
	}

	for (int i = 0; i < stracksb.size(); i++)
	{
		vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
		if (iter == dupb.end())
		{
			resb.push_back(stracksb[i]);
		}
	}
}

void BYTETrackerGrasper::linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
	vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b)
{
	if (cost_matrix.size() == 0)
	{
		for (int i = 0; i < cost_matrix_size; i++)
		{
			unmatched_a.push_back(i);
		}
		for (int i = 0; i < cost_matrix_size_size; i++)
		{
			unmatched_b.push_back(i);
		}
		return;
	}

	vector<int> rowsol; vector<int> colsol;
	float c = lapjv(cost_matrix, rowsol, colsol, true, thresh);
	for (int i = 0; i < rowsol.size(); i++)
	{
		if (rowsol[i] >= 0)
		{
			vector<int> match;
			match.push_back(i);
			match.push_back(rowsol[i]);
			matches.push_back(match);
		}
		else
		{
			unmatched_a.push_back(i);
		}
	}

	for (int i = 0; i < colsol.size(); i++)
	{
		if (colsol[i] < 0)
		{
			unmatched_b.push_back(i);
		}
	}
}

vector<vector<float> > BYTETrackerGrasper::norms(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs)
{
	vector<vector<float> > distances;
	if (atlbrs.size()*btlbrs.size() == 0)
		return distances;

	distances.resize(atlbrs.size());
	for (int i = 0; i < distances.size(); i++)
	{
		distances[i].resize(btlbrs.size());
	}

	//distances
	for (int k = 0; k < btlbrs.size(); k++)
	{
		//vector<float> ious_tmp;
		//float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
		for (int n = 0; n < atlbrs.size(); n++)
		{
			distances[n][k]=(640-sqrt((btlbrs[k][0]-atlbrs[n][0])*(btlbrs[k][0]-atlbrs[n][0])
						       +(btlbrs[k][1]-atlbrs[n][1])*(btlbrs[k][1]-atlbrs[n][1])))/640;
			// float iw = min(atlbrs[n][2], btlbrs[k][2]) - max(atlbrs[n][0], btlbrs[k][0]) + 1;
			// if (iw > 0)
			// {
			// 	float ih = min(atlbrs[n][3], btlbrs[k][3]) - max(atlbrs[n][1], btlbrs[k][1]) + 1;
			// 	if(ih > 0)
			// 	{
			// 		float ua = (atlbrs[n][2] - atlbrs[n][0] + 1)*(atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
			// 		ious[n][k] = iw * ih / ua;
			// 	}
			// 	else
			// 	{
			// 		ious[n][k] = 0.0;
			// 	}
			// }
			// else
			// {
			// 	ious[n][k] = 0.0;
			// }
		}
	}

	return distances;
}

vector<vector<float> > BYTETrackerGrasper::ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs)
{
	vector<vector<float> > ious;
	if (atlbrs.size()*btlbrs.size() == 0)
		return ious;

	ious.resize(atlbrs.size());
	for (int i = 0; i < ious.size(); i++)
	{
		ious[i].resize(btlbrs.size());
	}

	//bbox_ious
	for (int k = 0; k < btlbrs.size(); k++)
	{
		vector<float> ious_tmp;
		float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
		for (int n = 0; n < atlbrs.size(); n++)
		{
			float iw = min(atlbrs[n][2], btlbrs[k][2]) - max(atlbrs[n][0], btlbrs[k][0]) + 1;
			if (iw > 0)
			{
				float ih = min(atlbrs[n][3], btlbrs[k][3]) - max(atlbrs[n][1], btlbrs[k][1]) + 1;
				if(ih > 0)
				{
					float ua = (atlbrs[n][2] - atlbrs[n][0] + 1)*(atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
					ious[n][k] = iw * ih / ua;
				}
				else
				{
					ious[n][k] = 0.0;
				}
			}
			else
			{
				ious[n][k] = 0.0;
			}
		}
	}

	return ious;
}

// vector<vector<float> > BYTETrackerGrasper::norm_distance(vector<STrackGrasper*> &atracks, vector<STrackGrasper> &btracks, int &dist_size, int &dist_size_size)
// {
// 	vector<vector<float> > cost_matrix;
// 	if (atracks.size() * btracks.size() == 0)
// 	{
// 		dist_size = atracks.size();
// 		dist_size_size = btracks.size();
// 		return cost_matrix;
// 	}
// 	vector<vector<float> > atlbrs, btlbrs;
// 	for (int i = 0; i < atracks.size(); i++)
// 	{
// 		atlbrs.push_back(atracks[i]->xy);
// 	}
// 	for (int i = 0; i < btracks.size(); i++)
// 	{
// 		btlbrs.push_back(btracks[i].xy);
// 	}

// 	dist_size = atracks.size();
// 	dist_size_size = btracks.size();

// 	vector<vector<float> > _distances = norms(atlbrs, btlbrs);
	
// 	for (int i = 0; i < _distances.size();i++)
// 	{
// 		vector<float> _iou;
// 		for (int j = 0; j < _distances[i].size(); j++)
// 		{
// 			_iou.push_back(1 - _distances[i][j]);
// 		}
// 		cost_matrix.push_back(_iou);
// 	}

// 	return cost_matrix;
// }

// vector<vector<float> > BYTETrackerGrasper::iou_distance(vector<STrackGrasper*> &atracks, vector<STrackGrasper> &btracks, int &dist_size, int &dist_size_size)
// {
// 	vector<vector<float> > cost_matrix;
// 	if (atracks.size() * btracks.size() == 0)
// 	{
// 		dist_size = atracks.size();
// 		dist_size_size = btracks.size();
// 		return cost_matrix;
// 	}
// 	vector<vector<float> > atlbrs, btlbrs;
// 	for (int i = 0; i < atracks.size(); i++)
// 	{
// 		atlbrs.push_back(atracks[i]->tlbr);
// 	}
// 	for (int i = 0; i < btracks.size(); i++)
// 	{
// 		btlbrs.push_back(btracks[i].tlbr);
// 	}

// 	dist_size = atracks.size();
// 	dist_size_size = btracks.size();

// 	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	
// 	for (int i = 0; i < _ious.size();i++)
// 	{
// 		vector<float> _iou;
// 		for (int j = 0; j < _ious[i].size(); j++)
// 		{
// 			_iou.push_back(1 - _ious[i][j]);
// 		}
// 		cost_matrix.push_back(_iou);
// 	}

// 	return cost_matrix;
// }

vector<vector<float> > BYTETrackerGrasper::avg_distance(vector<STrackGrasper*> &atracks, vector<STrackGrasper> &btracks, int &dist_size, int &dist_size_size)
{
	vector<vector<float> > cost_matrix;
	if (atracks.size() * btracks.size() == 0)
	{
		dist_size = atracks.size();
		dist_size_size = btracks.size();
		return cost_matrix;
	}
	vector<vector<float> > atlbrs, btlbrs;
	vector<vector<float> > alocators, blocators, ajoints, bjoints, alefts, blefts, arights, brights;
	for (int i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i]->tlbr);
		alocators.push_back(atracks[i]->locator);
		ajoints.push_back(atracks[i]->joint);
		alefts.push_back(atracks[i]->left);
		arights.push_back(atracks[i]->right);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
		blocators.push_back(btracks[i].locator);
		bjoints.push_back(btracks[i].joint);
		blefts.push_back(btracks[i].left);
		brights.push_back(btracks[i].right);
	}

	dist_size = atracks.size();
	dist_size_size = btracks.size();

	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	vector<vector<float> > _distances_locator = norms(alocators, blocators);
	vector<vector<float> > _distances_joint = norms(ajoints, bjoints);
	vector<vector<float> > _distances_left = norms(alefts, blefts);
	vector<vector<float> > _distances_right = norms(arights, brights);
	
	for (int i = 0; i < _ious.size();i++)
	{
		vector<float> _avg;
		for (int j = 0; j < _ious[i].size(); j++)
		{
			_avg.push_back(1 - (_ious[i][j]+_distances_locator[i][j]+_distances_joint[i][j]+_distances_left[i][j]+_distances_right[i][j])/5);
			// _avg.push_back(1 - _distances[i][j]);
		}
		cost_matrix.push_back(_avg);
	}

	return cost_matrix;
}

// vector<vector<float> > BYTETrackerGrasper::norm_distance(vector<STrackGrasper> &atracks, vector<STrackGrasper> &btracks)
// {
// 	vector<vector<float> > atlbrs, btlbrs;
// 	for (int i = 0; i < atracks.size(); i++)
// 	{
// 		atlbrs.push_back(atracks[i].xy);
// 	}
// 	for (int i = 0; i < btracks.size(); i++)
// 	{
// 		btlbrs.push_back(btracks[i].xy);
// 	}

// 	vector<vector<float> > _distances = norms(atlbrs, btlbrs);
// 	vector<vector<float> > cost_matrix;
// 	for (int i = 0; i < _distances.size(); i++)
// 	{
// 		vector<float> _iou;
// 		for (int j = 0; j < _distances[i].size(); j++)
// 		{
// 			_iou.push_back(1 - _distances[i][j]);
// 		}
// 		cost_matrix.push_back(_iou);
// 	}

// 	return cost_matrix;
// }

vector<vector<float> > BYTETrackerGrasper::avg_distance(vector<STrackGrasper> &atracks, vector<STrackGrasper> &btracks)
{
	vector<vector<float> > atlbrs, btlbrs;
	vector<vector<float> > alocators, blocators, ajoints, bjoints, alefts, blefts, arights, brights;
	for (int i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i].tlbr);
		alocators.push_back(atracks[i].locator);
		ajoints.push_back(atracks[i].joint);
		alefts.push_back(atracks[i].left);
		arights.push_back(atracks[i].right);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
		blocators.push_back(btracks[i].locator);
		bjoints.push_back(btracks[i].joint);
		blefts.push_back(btracks[i].left);
		brights.push_back(btracks[i].right);
	}

	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	vector<vector<float> > _distances_locator = norms(alocators, blocators);
	vector<vector<float> > _distances_joint = norms(ajoints, bjoints);
	vector<vector<float> > _distances_left = norms(alefts, blefts);
	vector<vector<float> > _distances_right = norms(arights, brights);
	vector<vector<float> > cost_matrix;
	for (int i = 0; i < _ious.size(); i++)
	{
		vector<float> _avg;
		for (int j = 0; j < _ious[i].size(); j++)
		{
			_avg.push_back(1 - (_ious[i][j]+_distances_locator[i][j]+_distances_joint[i][j]+_distances_left[i][j]+_distances_right[i][j])/5);
			// _avg.push_back(1 - _distances[i][j]);
		}
		cost_matrix.push_back(_avg);
	}

	return cost_matrix;
}

// vector<vector<float> > BYTETrackerGrasper::iou_distance(vector<STrackGrasper> &atracks, vector<STrackGrasper> &btracks)
// {
// 	vector<vector<float> > atlbrs, btlbrs;
// 	for (int i = 0; i < atracks.size(); i++)
// 	{
// 		atlbrs.push_back(atracks[i].tlbr);
// 	}
// 	for (int i = 0; i < btracks.size(); i++)
// 	{
// 		btlbrs.push_back(btracks[i].tlbr);
// 	}

// 	vector<vector<float> > _ious = ious(atlbrs, btlbrs);
// 	vector<vector<float> > cost_matrix;
// 	for (int i = 0; i < _ious.size(); i++)
// 	{
// 		vector<float> _iou;
// 		for (int j = 0; j < _ious[i].size(); j++)
// 		{
// 			_iou.push_back(1 - _ious[i][j]);
// 		}
// 		cost_matrix.push_back(_iou);
// 	}

// 	return cost_matrix;
// }

double BYTETrackerGrasper::lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
	bool extend_cost, float cost_limit, bool return_cost)
{
	vector<vector<float> > cost_c;
	cost_c.assign(cost.begin(), cost.end());

	vector<vector<float> > cost_c_extended;

	int n_rows = cost.size();
	int n_cols = cost[0].size();
	rowsol.resize(n_rows);
	colsol.resize(n_cols);

	int n = 0;
	if (n_rows == n_cols)
	{
		n = n_rows;
	}
	else
	{
		if (!extend_cost)
		{
			cout << "set extend_cost=True" << endl;
			system("pause");
			exit(0);
		}
	}
		
	if (extend_cost || cost_limit < LONG_MAX)
	{
		n = n_rows + n_cols;
		cost_c_extended.resize(n);
		for (int i = 0; i < cost_c_extended.size(); i++)
			cost_c_extended[i].resize(n);

		if (cost_limit < LONG_MAX)
		{
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_limit / 2.0;
				}
			}
		}
		else
		{
			float cost_max = -1;
			for (int i = 0; i < cost_c.size(); i++)
			{
				for (int j = 0; j < cost_c[i].size(); j++)
				{
					if (cost_c[i][j] > cost_max)
						cost_max = cost_c[i][j];
				}
			}
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_max + 1;
				}
			}
		}

		for (int i = n_rows; i < cost_c_extended.size(); i++)
		{
			for (int j = n_cols; j < cost_c_extended[i].size(); j++)
			{
				cost_c_extended[i][j] = 0;
			}
		}
		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				cost_c_extended[i][j] = cost_c[i][j];
			}
		}

		cost_c.clear();
		cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
	}

	double **cost_ptr;
	cost_ptr = new double *[sizeof(double *) * n];
	for (int i = 0; i < n; i++)
		cost_ptr[i] = new double[sizeof(double) * n];

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cost_ptr[i][j] = cost_c[i][j];
		}
	}

	int* x_c = new int[sizeof(int) * n];
	int *y_c = new int[sizeof(int) * n];

	int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
	if (ret != 0)
	{
		cout << "Calculate Wrong!" << endl;
		system("pause");
		exit(0);
	}

	double opt = 0.0;

	if (n != n_rows)
	{
		for (int i = 0; i < n; i++)
		{
			if (x_c[i] >= n_cols)
				x_c[i] = -1;
			if (y_c[i] >= n_rows)
				y_c[i] = -1;
		}
		for (int i = 0; i < n_rows; i++)
		{
			rowsol[i] = x_c[i];
		}
		for (int i = 0; i < n_cols; i++)
		{
			colsol[i] = y_c[i];
		}

		if (return_cost)
		{
			for (int i = 0; i < rowsol.size(); i++)
			{
				if (rowsol[i] != -1)
				{
					//cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
					opt += cost_ptr[i][rowsol[i]];
				}
			}
		}
	}
	else if (return_cost)
	{
		for (int i = 0; i < rowsol.size(); i++)
		{
			opt += cost_ptr[i][rowsol[i]];
		}
	}

	for (int i = 0; i < n; i++)
	{
		delete[]cost_ptr[i];
	}
	delete[]cost_ptr;
	delete[]x_c;
	delete[]y_c;

	return opt;
}

Scalar BYTETrackerGrasper::get_color(int idx)
{
	idx += 3;
	return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}

// ====================================
// modified Bean 3D

vector<STrackBean3D*> BYTETrackerBean3D::joint_stracks(vector<STrackBean3D*> &tlista, vector<STrackBean3D> &tlistb)
{
	map<int, int> exists;
	vector<STrackBean3D*> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i]->track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(&tlistb[i]);
		}
	}
	return res;
}

vector<STrackBean3D> BYTETrackerBean3D::joint_stracks(vector<STrackBean3D> &tlista, vector<STrackBean3D> &tlistb)
{
	map<int, int> exists;
	vector<STrackBean3D> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i].track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(tlistb[i]);
		}
	}
	return res;
}

vector<STrackBean3D> BYTETrackerBean3D::sub_stracks(vector<STrackBean3D> &tlista, vector<STrackBean3D> &tlistb)
{
	map<int, STrackBean3D> stracks;
	for (int i = 0; i < tlista.size(); i++)
	{
		stracks.insert(pair<int, STrackBean3D>(tlista[i].track_id, tlista[i]));
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (stracks.count(tid) != 0)
		{
			stracks.erase(tid);
		}
	}

	vector<STrackBean3D> res;
	std::map<int, STrackBean3D>::iterator  it;
	for (it = stracks.begin(); it != stracks.end(); ++it)
	{
		res.push_back(it->second);
	}

	return res;
}

void BYTETrackerBean3D::remove_duplicate_stracks(vector<STrackBean3D> &resa, vector<STrackBean3D> &resb, vector<STrackBean3D> &stracksa, vector<STrackBean3D> &stracksb)
{
	vector<vector<float> > pdist = avg_distance(stracksa, stracksb);
	vector<pair<int, int> > pairs;
	for (int i = 0; i < pdist.size(); i++)
	{
		for (int j = 0; j < pdist[i].size(); j++)
		{
			if (pdist[i][j] < 0.15)
			{
				pairs.push_back(pair<int, int>(i, j));
			}
		}
	}

	vector<int> dupa, dupb;
	for (int i = 0; i < pairs.size(); i++)
	{
		int timep = stracksa[pairs[i].first].frame_id - stracksa[pairs[i].first].start_frame;
		int timeq = stracksb[pairs[i].second].frame_id - stracksb[pairs[i].second].start_frame;
		if (timep > timeq)
			dupb.push_back(pairs[i].second);
		else
			dupa.push_back(pairs[i].first);
	}

	for (int i = 0; i < stracksa.size(); i++)
	{
		vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
		if (iter == dupa.end())
		{
			resa.push_back(stracksa[i]);
		}
	}

	for (int i = 0; i < stracksb.size(); i++)
	{
		vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
		if (iter == dupb.end())
		{
			resb.push_back(stracksb[i]);
		}
	}
}

void BYTETrackerBean3D::linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
	vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b)
{
	if (cost_matrix.size() == 0)
	{
		for (int i = 0; i < cost_matrix_size; i++)
		{
			unmatched_a.push_back(i);
		}
		for (int i = 0; i < cost_matrix_size_size; i++)
		{
			unmatched_b.push_back(i);
		}
		return;
	}

	vector<int> rowsol; vector<int> colsol;
	float c = lapjv(cost_matrix, rowsol, colsol, true, thresh);
	for (int i = 0; i < rowsol.size(); i++)
	{
		if (rowsol[i] >= 0)
		{
			vector<int> match;
			match.push_back(i);
			match.push_back(rowsol[i]);
			matches.push_back(match);
		}
		else
		{
			unmatched_a.push_back(i);
		}
	}

	for (int i = 0; i < colsol.size(); i++)
	{
		if (colsol[i] < 0)
		{
			unmatched_b.push_back(i);
		}
	}
}

vector<vector<float> > BYTETrackerBean3D::norms(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs)
{
	vector<vector<float> > distances;
	if (atlbrs.size()*btlbrs.size() == 0)
		return distances;

	distances.resize(atlbrs.size());
	for (int i = 0; i < distances.size(); i++)
	{
		distances[i].resize(btlbrs.size());
	}

	//distances
	for (int k = 0; k < btlbrs.size(); k++)
	{
		//vector<float> ious_tmp;
		//float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
		for (int n = 0; n < atlbrs.size(); n++)
		{
			distances[n][k]=exp(-sqrt((btlbrs[k][0]-atlbrs[n][0])*(btlbrs[k][0]-atlbrs[n][0])
						       +(btlbrs[k][1]-atlbrs[n][1])*(btlbrs[k][1]-atlbrs[n][1])));
		}
	}

	return distances;
}

// vector<vector<float> > BYTETrackerBean3D::ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs)
// {
// 	vector<vector<float> > ious;
// 	if (atlbrs.size()*btlbrs.size() == 0)
// 		return ious;

// 	ious.resize(atlbrs.size());
// 	for (int i = 0; i < ious.size(); i++)
// 	{
// 		ious[i].resize(btlbrs.size());
// 	}

// 	//bbox_ious
// 	for (int k = 0; k < btlbrs.size(); k++)
// 	{
// 		vector<float> ious_tmp;
// 		float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
// 		for (int n = 0; n < atlbrs.size(); n++)
// 		{
// 			float iw = min(atlbrs[n][2], btlbrs[k][2]) - max(atlbrs[n][0], btlbrs[k][0]) + 1;
// 			if (iw > 0)
// 			{
// 				float ih = min(atlbrs[n][3], btlbrs[k][3]) - max(atlbrs[n][1], btlbrs[k][1]) + 1;
// 				if(ih > 0)
// 				{
// 					float ua = (atlbrs[n][2] - atlbrs[n][0] + 1)*(atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
// 					ious[n][k] = iw * ih / ua;
// 				}
// 				else
// 				{
// 					ious[n][k] = 0.0;
// 				}
// 			}
// 			else
// 			{
// 				ious[n][k] = 0.0;
// 			}
// 		}
// 	}

// 	return ious;
// }


vector<vector<float> > BYTETrackerBean3D::avg_distance(vector<STrackBean3D*> &atracks, vector<STrackBean3D> &btracks, int &dist_size, int &dist_size_size)
{
	vector<vector<float> > cost_matrix;
	if (atracks.size() * btracks.size() == 0)
	{
		dist_size = atracks.size();
		dist_size_size = btracks.size();
		return cost_matrix;
	}
	vector<vector<float> > atlbrs, btlbrs;
	vector<vector<float> > axys, bxys;
	for (int i = 0; i < atracks.size(); i++)
	{
		axys.push_back(atracks[i]->xyz);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		bxys.push_back(btracks[i].xyz);
	}

	dist_size = atracks.size();
	dist_size_size = btracks.size();

	vector<vector<float> > _distances = norms(axys, bxys);
	
	for (int i = 0; i < _distances.size();i++)
	{
		vector<float> _avg;
		for (int j = 0; j < _distances[i].size(); j++)
		{
			_avg.push_back(1 - _distances[i][j]);
			// _avg.push_back(1 - _distances[i][j]);
		}
		cost_matrix.push_back(_avg);
	}

	return cost_matrix;
}

vector<vector<float> > BYTETrackerBean3D::avg_distance(vector<STrackBean3D> &atracks, vector<STrackBean3D> &btracks)
{
	vector<vector<float> > atlbrs, btlbrs;
	vector<vector<float> > axys, bxys;
	for (int i = 0; i < atracks.size(); i++)
	{
		// atlbrs.push_back(atracks[i].tlbr);
		axys.push_back(atracks[i].xyz);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		// btlbrs.push_back(btracks[i].tlbr);
		bxys.push_back(btracks[i].xyz);
	}

	// vector<vector<float> > _ious = ious(atlbrs, btlbrs);
	vector<vector<float> > _distances = norms(axys, bxys);
	vector<vector<float> > cost_matrix;
	for (int i = 0; i < _distances.size(); i++)
	{
		vector<float> _avg;
		for (int j = 0; j < _distances[i].size(); j++)
		{
			_avg.push_back(1 - _distances[i][j]);
			// _avg.push_back(1 - _distances[i][j]);
		}
		cost_matrix.push_back(_avg);
	}

	return cost_matrix;
}

double BYTETrackerBean3D::lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
	bool extend_cost, float cost_limit, bool return_cost)
{
	vector<vector<float> > cost_c;
	cost_c.assign(cost.begin(), cost.end());

	vector<vector<float> > cost_c_extended;

	int n_rows = cost.size();
	int n_cols = cost[0].size();
	rowsol.resize(n_rows);
	colsol.resize(n_cols);

	int n = 0;
	if (n_rows == n_cols)
	{
		n = n_rows;
	}
	else
	{
		if (!extend_cost)
		{
			cout << "set extend_cost=True" << endl;
			system("pause");
			exit(0);
		}
	}
		
	if (extend_cost || cost_limit < LONG_MAX)
	{
		n = n_rows + n_cols;
		cost_c_extended.resize(n);
		for (int i = 0; i < cost_c_extended.size(); i++)
			cost_c_extended[i].resize(n);

		if (cost_limit < LONG_MAX)
		{
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_limit / 2.0;
				}
			}
		}
		else
		{
			float cost_max = -1;
			for (int i = 0; i < cost_c.size(); i++)
			{
				for (int j = 0; j < cost_c[i].size(); j++)
				{
					if (cost_c[i][j] > cost_max)
						cost_max = cost_c[i][j];
				}
			}
			for (int i = 0; i < cost_c_extended.size(); i++)
			{
				for (int j = 0; j < cost_c_extended[i].size(); j++)
				{
					cost_c_extended[i][j] = cost_max + 1;
				}
			}
		}

		for (int i = n_rows; i < cost_c_extended.size(); i++)
		{
			for (int j = n_cols; j < cost_c_extended[i].size(); j++)
			{
				cost_c_extended[i][j] = 0;
			}
		}
		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				cost_c_extended[i][j] = cost_c[i][j];
			}
		}

		cost_c.clear();
		cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
	}

	double **cost_ptr;
	cost_ptr = new double *[sizeof(double *) * n];
	for (int i = 0; i < n; i++)
		cost_ptr[i] = new double[sizeof(double) * n];

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cost_ptr[i][j] = cost_c[i][j];
		}
	}

	int* x_c = new int[sizeof(int) * n];
	int *y_c = new int[sizeof(int) * n];

	int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
	if (ret != 0)
	{
		cout << "Calculate Wrong!" << endl;
		system("pause");
		exit(0);
	}

	double opt = 0.0;

	if (n != n_rows)
	{
		for (int i = 0; i < n; i++)
		{
			if (x_c[i] >= n_cols)
				x_c[i] = -1;
			if (y_c[i] >= n_rows)
				y_c[i] = -1;
		}
		for (int i = 0; i < n_rows; i++)
		{
			rowsol[i] = x_c[i];
		}
		for (int i = 0; i < n_cols; i++)
		{
			colsol[i] = y_c[i];
		}

		if (return_cost)
		{
			for (int i = 0; i < rowsol.size(); i++)
			{
				if (rowsol[i] != -1)
				{
					//cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
					opt += cost_ptr[i][rowsol[i]];
				}
			}
		}
	}
	else if (return_cost)
	{
		for (int i = 0; i < rowsol.size(); i++)
		{
			opt += cost_ptr[i][rowsol[i]];
		}
	}

	for (int i = 0; i < n; i++)
	{
		delete[]cost_ptr[i];
	}
	delete[]cost_ptr;
	delete[]x_c;
	delete[]y_c;

	return opt;
}

Scalar BYTETrackerBean3D::get_color(int idx)
{
	idx += 3;
	return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}
