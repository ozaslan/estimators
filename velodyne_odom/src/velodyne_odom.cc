#include "velodyne_odom.hh"
#include "utils.hh"

/*
   -- Implement overlapping-spheres for loop closure
   -- Integrate ndt fit score into factor sigmas
 */

using namespace std;

int VelodyneOdom::VelodyneOdomParams::print(){
	cout << "VelodyneOdomParams -----------------------" << endl;
	cout << "\t voxel_leaf_size = [" << voxel_leaf_size[0] << ", " 
		<< voxel_leaf_size[1] << ", "
		<< voxel_leaf_size[2] << "]" << endl;
	cout << "\t local_map_dims = [" << local_map_dims[0] << ", " 
		<< local_map_dims[1] << ", "
		<< local_map_dims[2] << "]" << endl;
	cout << "\t local_map_max_points = [" << local_map_max_points << "]" << endl;
	cout << "\t ndt_eps --------------- = " << ndt_eps << endl;
	cout << "\t ndt_res --------------- = " << ndt_res << endl;
	cout << "\t ndt_max_iter ---------- = " << ndt_max_iter << endl;
	cout << "\t ndt_step_size --------- = " << ndt_step_size << endl;
	cout << "\t ndt_fitness_score_thres = " << ndt_fitness_score_thres << endl;
	cout << "\t batch_ndt_eps --------- = " << batch_ndt_eps << endl;
	cout << "\t batch_ndt_res --------- = " << batch_ndt_res << endl;
	cout << "\t batch_ndt_max_iter ---- = " << batch_ndt_max_iter << endl;
	cout << "\t batch_ndt_step_size --  = " << batch_ndt_step_size << endl;
	cout << "\t init_keyframe_trans_thres = " << init_keyframe_trans_thres << endl;
	cout << "\t init_keyframe_rot_thres - = " << init_keyframe_rot_thres << endl;
	cout << "------------------------------------------" << endl;
	return 0;
}

void VelodyneOdom::_initialize(){
	_voxel_filter.setLeafSize (_params.voxel_leaf_size[0],
			_params.voxel_leaf_size[1],
			_params.voxel_leaf_size[2]);
	_voxel_filter.setMinimumPointsNumberPerVoxel(1);

	_local_map_bbox(0) = _params.local_map_dims[0];
	_local_map_bbox(1) = _params.local_map_dims[1];
	_local_map_bbox(2) = _params.local_map_dims[2];

	_batch_optimizing = false;

	_map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_local_map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	_curr_keyframe_ind = -1;
	_keyframes.clear();
	_keyframe_poses.clear();

	_keyframe_connectivity.clear();
	_pose_keyframes_distances.clear();

	_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_filtered_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_aligned_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	_pc_pose.setIdentity();
	_cov.setIdentity();

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	_ndt.setTransformationEpsilon (_params.ndt_eps);
	_batch_ndt.setTransformationEpsilon (_params.batch_ndt_eps);
	// Setting maximum step size for More-Thuente line search.
	_ndt.setStepSize (_params.ndt_step_size);
	_batch_ndt.setStepSize (_params.batch_ndt_step_size);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	_ndt.setResolution (_params.ndt_res);
	_batch_ndt.setResolution (_params.batch_ndt_res);
	// Setting max er of registration iterations.
	_ndt.setMaximumIterations (_params.ndt_max_iter);
	_batch_ndt.setMaximumIterations (_params.batch_ndt_max_iter);

	_gtsam_prior_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
	//_params.print();
}

VelodyneOdom::VelodyneOdom(){
	_initialize();
}

VelodyneOdom::VelodyneOdom(const VelodyneOdomParams &params){
	_params = params;
	_initialize();
}

double VelodyneOdom::_pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double trans_thres, double rot_thres){
	Eigen::Matrix4d dpose = pose1.inverse() * pose2;
	Eigen::Vector3d aaxis = utils::trans::dcm2aaxis(utils::trans::cancel_yaw(Eigen::Matrix3d(dpose.topLeftCorner<3, 3>())));
	return (dpose.topRightCorner<3, 1>().norm() / trans_thres) + (aaxis.norm() / rot_thres);
}

double VelodyneOdom::_pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2){
	return _pose_distance(pose1, pose2, _params.init_keyframe_trans_thres, _params.init_keyframe_rot_thres);
}

bool VelodyneOdom::_push_new_keyframe_required(const Eigen::Matrix4d &curr_pose, int key_frame_ind){
	ASSERT(key_frame_ind < (int)_keyframes.size(), "'key_frame_ind < _keyframes.size()' should hold ");

	//cout << "Need to push new keyframe ? : " << _pose_distance(curr_pose, _keyframe_poses[key_frame_ind]) << endl;
	return _pose_distance(curr_pose, _keyframe_poses[key_frame_ind]) >= 1.0;
}

int VelodyneOdom::_find_closest_keyframe(const Eigen::Matrix4d &curr_pose){
	double best_keyframe_dist = DBL_MAX;
	int    best_keyframe_ind  = -1;
	for(int i = _keyframe_poses.size() - 1; i >= 0 ; i--){
		double curr_keyframe_dist = _pose_distance(curr_pose, _keyframe_poses[i]);
		if(curr_keyframe_dist < best_keyframe_dist){
			best_keyframe_dist = curr_keyframe_dist;
			best_keyframe_ind  = i;
		}
	}
	return best_keyframe_ind;	
}

int VelodyneOdom::_batch_optimize(int num_keyframes){
	vector<int> keyframe_indices;
	for(int i = _keyframes.size() - num_keyframes ; i < (int)_keyframes.size() ; i++)
		keyframe_indices.push_back(i);
	return _batch_optimize(keyframe_indices);
}

int VelodyneOdom::_batch_optimize(int start_keyframe_ind, int end_keyframes_ind){
	vector<int> keyframe_indices;
	for(int i = start_keyframe_ind ; i < end_keyframes_ind ; i++)
		keyframe_indices.push_back(i);
	return _batch_optimize(keyframe_indices);
}

int VelodyneOdom::_batch_optimize(const vector<int> &keyframe_indices){
	ASSERT(false, "To be implemeted...");
	return 0;
}

int VelodyneOdom::_cancel_batch_optimization(bool undo_changes){
	ASSERT(false, "To be implemeted...");
	return 0;
}

int VelodyneOdom::_trim_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const Eigen::Vector3d &pos, const Eigen::Vector3d &bbox){
	_pass_filter.setInputCloud(pc);
	_pass_filter.setFilterFieldName("x");
	_pass_filter.setFilterLimits(pos(0) - bbox(0)/2, pos(0) + bbox(0)/2);
	_pass_filter.filter(*pc);
	_pass_filter.setFilterFieldName("y");
	_pass_filter.setFilterLimits(pos(1) - bbox(1)/2, pos(1) + bbox(1)/2);
	_pass_filter.filter(*pc);
	_pass_filter.setFilterFieldName("z");
	_pass_filter.setFilterLimits(pos(2) - bbox(2)/2, pos(2) + bbox(2)/2);
	_pass_filter.filter(*pc);

	return pc->points.size();
}

int VelodyneOdom::_build_local_map(const Eigen::Matrix4d &curr_pose){
	// Generate the sorted distances vector
	_find_pose_keyframes_distances(curr_pose);
	
	/*
	cout << "-------------------------------------------------" << endl;
	cout << "Pose-Keyframe Distances " << endl;
	for(int i = 0 ; i < (int) _pose_keyframes_distances.size() ; i++){
		cout << "\t" << _pose_keyframes_distances[i].first << " - " << _pose_keyframes_distances[i].second << endl;
	}

	cout << "Previous Pose-Keyframe Distances " << endl;
	for(int i = 0 ; i < (int) _prev_pose_keyframes_distances.size() ; i++){
		cout << "\t" << _prev_pose_keyframes_distances[i].first << " - " << _prev_pose_keyframes_distances[i].second << endl;
	}
	*/

	// If the first 5 keyframes are the same between the current and 
	// the previous sorted list, do not rebuild '_local_map'
	if(_prev_pose_keyframes_distances.size() == _pose_keyframes_distances.size()){
		bool diff_exists = false;
		for(int i = 0 ; i < 3 && i < (int)_pose_keyframes_distances.size() ; i++)
			diff_exists |= _prev_pose_keyframes_distances[i].first != _pose_keyframes_distances[i].first;
		if(diff_exists == false){
			//cout << "There is no difference in pose-keyframe distances. No need to rebuild the local map" << endl;
			return 0;
		}
	}

	//cout << "Need to rebuild the local map" << endl;

	_local_map->clear();

	for(int i = 0 ; i < (int)_pose_keyframes_distances.size() ; i++){
		int ind = _pose_keyframes_distances[i].first;

		// Trim the keyframe point cloud before adding to the local map.
		pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed_keyframe_pointcloud = _keyframes[ind]->makeShared();
		_trim_pointcloud(trimmed_keyframe_pointcloud, curr_pose.topRightCorner<3, 1>(), _local_map_bbox);
		*_local_map += *trimmed_keyframe_pointcloud;

		if(_local_map->points.size() > _params.local_map_max_points){
			_voxel_filter.setInputCloud(_local_map);
			_voxel_filter.filter(*_local_map);
			// If local map has too many points even after filtering, stop expanding it
			if(_local_map->points.size() > _params.local_map_max_points)
				break;
		}
	}

	//cout << "Size of the new local map : <" << _local_map->points.size() << ">" << endl;

	_ndt.setInputTarget(_local_map);
	_prev_pose_keyframes_distances = _pose_keyframes_distances;
}

int VelodyneOdom::_find_pose_keyframes_distances(const Eigen::Matrix4d &curr_pose){

	// ### check if the closest keyframe set has changed. Otherwise do not rebuild the local map	
	_pose_keyframes_distances.resize(_keyframes.size());
	for(int i = _keyframe_poses.size() - 1; i >= 0 ; i--){
		_pose_keyframes_distances[i].first = i;
		_pose_keyframes_distances[i].second = _pose_distance(curr_pose, _keyframe_poses[i]);
	}
	std::sort(_pose_keyframes_distances.begin(), _pose_keyframes_distances.end(),
			[&](std::pair<int, double> i1, std::pair<int, double> i2){return i1.second < i2.second;});

	/*
	for(int i = 0 ; i < _keyframes.size() ; i++)
		cout << _pose_keyframes_distances[i].first << " - " << _pose_keyframes_distances[i].second << endl;
	*/
	return 0;
}

int VelodyneOdom::push_pc(const sensor_msgs::PointCloud2 &pc){
	pcl::fromROSMsg(pc, *_pc);	
	
	// If align(...) has never been called, reinitialize the map and keyframes
	if(_keyframe_poses.empty()){
		_keyframes.clear();
		_keyframes.push_back(_pc->makeShared());
		_map = _pc->makeShared();
		_curr_keyframe_ind = 0;
	}
	return 0;
}

int VelodyneOdom::align(){
	Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
	return align(init_pose);
}

int VelodyneOdom::align(const Eigen::Vector3d &init_pos){
	Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
	init_pose.topRightCorner<3, 1>() = init_pos;
	return align(init_pose);
}

int VelodyneOdom::align(const Eigen::Matrix3d &init_dcm){
	Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
	init_pose.topLeftCorner<3, 3>() = init_dcm;
	return align(init_pose);
}

int VelodyneOdom::align(const Eigen::Matrix4d &init_pose){

	if(_keyframe_poses.empty()){
		_keyframe_poses.push_back(init_pose);
		_gtsam_graph_with_prior.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(_keyframe_poses[0]), _gtsam_prior_model));
	}

	utils::Timer timer(true, "align-");
	timer.tic();
	timer.toc("1"); timer.tic();

	// Downsample the input point cloud using voxel grids
	// ### remember that we were using the approximate grid filter before!!!
	_voxel_filter.setInputCloud(_pc);
	_voxel_filter.filter(*_filtered_pc);

	timer.toc("2"); timer.tic();
	// Find the closest keyframes to build '_local_map' with respect to 
	// which the odometry is calculated
	_build_local_map(init_pose);

	timer.toc("3"); timer.tic();
	// Align the input point cloud to the keyframe
	_ndt.setInputSource(_filtered_pc);
	timer.toc("3.1"); timer.tic();
	_ndt.align(*_aligned_pc, init_pose.cast<float>());

	cout << "-- Size of the new local map : <" << _local_map->points.size() << ">" << endl;
	timer.toc("4"); timer.tic();

	double fitness_score = _ndt.getFitnessScore();

	cout << "--- Fitness score : " << fitness_score << endl;
	cout << "--- Has converged : " << (_ndt.hasConverged() ? "TRUE" : "FALSE") << endl;

	if(_ndt.hasConverged() == false || fitness_score > _params.ndt_fitness_score_thres){
		// Set covariance to some large matrix
		_cov = Eigen::Matrix6d::Identity() * 1e12;
		return -1;
	}

	// Fit some unrealistic covariance
	_cov = Eigen::Matrix6d::Identity();
	_cov.topLeftCorner<3, 3>()     *= fitness_score * 1e-2;
	_cov.bottomRightCorner<3, 3>() *= fitness_score * 1e-3;

	// Record the final robot state
	_pc_pose = _ndt.getFinalTransformation().cast<double>();

	// Add new keyframes is alignment is succesful and current pose farther from 
	// the keyframe than a threshold 
	_curr_keyframe_ind = _find_closest_keyframe(_pc_pose);
	
	timer.toc("5"); timer.tic();
	//cout << "curr keyframe ind = " << _curr_keyframe_ind << endl;
	if(_push_new_keyframe_required(_pc_pose, _curr_keyframe_ind)){
		// Use the original (non-downsampled point cloud)
		
		pcl::transformPointCloud(*_pc, *_aligned_pc, _ndt.getFinalTransformation());

		// ### Do this is another thread
		// ### You might want to use _map->points.reserve(...)
	
		/*	
		if(fitness_score < _params.ndt_fitness_score_thres / 10){
			*_map += *_aligned_pc;
			_voxel_filter.setInputCloud(_map);
			_voxel_filter.filter(*_map);
		}
		*/
		// Add new keyframe
		_keyframes.push_back(_aligned_pc->makeShared());
		_keyframe_poses.push_back(_pc_pose);
		timer.toc("5.1"); timer.tic();
	}

	return 0;
}





// Sample code for GTSAM
/*
   Eigen::Matrix4d dpose = _keyframe_poses[_curr_keyframe_ind].inverse() * _pc_pose;

   _gtsam_graph_with_prior.add(gtsam::BetweenFactor<gtsam::Pose3>(
   _curr_keyframe_ind, (int)_keyframes.size() - 1,
   gtsam::Pose3(dpose),
   _gtsam_prior_model));

// Keep track of added factors
_pose_graph_edges.push_back(std::pair<int, int>(_curr_keyframe_ind, (int)(_keyframes.size() - 1)));
 */

/*
// Check if the robot is close to a keyframe visited earlier, "Loop Closure!!!"
if(((int)_keyframes.size() - 1) - _curr_keyframe_ind > 1){
gtsam::Values initial;
for(int i = 0 ; i < (int)_keyframes.size() ; i++)
initial.insert(i, gtsam::Pose3(_keyframe_poses[i]));

gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(_gtsam_graph_with_prior, initial).optimize();
for(int i = 0 ; i < (int)_keyframe_poses.size() ; i++){
gtsam::Pose3 pose = result.at<gtsam::Pose3>(i);
//cout << "Keyframe Pose[" << i << "] before : " << endl << _keyframe_poses[i] << endl;
_keyframe_poses[i].topLeftCorner<3, 3>() = pose.rotation().matrix();
_keyframe_poses[i].topRightCorner<3, 1>() = pose.translation();
//cout << "Keyframe Pose[" << i << "] after  : " << endl << _keyframe_poses[i] << endl;
}
//exit(0);
}
 */


