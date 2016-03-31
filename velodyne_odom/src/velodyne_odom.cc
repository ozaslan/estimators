#include "velodyne_odom.hh"
#include "utils.hh"

using namespace std;

int VelodyneOdom::VelodyneOdomParams::print(){
	cout << "VelodyneOdomParams -----------------------" << endl;
	cout << "\t approximate_voxel_leaf_size = [" << approximate_voxel_leaf_size[0] << ", " 
											  << approximate_voxel_leaf_size[1] << ", "
											  << approximate_voxel_leaf_size[2] << "]" << endl;
	cout << "\t ndt_eps ------ = " << ndt_eps << endl;
	cout << "\t ndt_res ------ = " << ndt_res << endl;
	cout << "\t ndt_max_iter - = " << ndt_max_iter << endl;
	cout << "\t ndt_step_size  = " << ndt_step_size << endl;
	cout << "\t batch_ndt_eps ------ = " << batch_ndt_eps << endl;
	cout << "\t batch_ndt_res ------ = " << batch_ndt_res << endl;
	cout << "\t batch_ndt_max_iter - = " << batch_ndt_max_iter << endl;
	cout << "\t batch_ndt_step_size  = " << batch_ndt_step_size << endl;
	cout << "\t trans_offset_weight  = " << trans_offset_weight << endl;
	cout << "\t rot_offset_weight -- = [" << rot_offset_weight[0] << ", " 
									   << rot_offset_weight[1] << ", "
									   << rot_offset_weight[2] << "]" << endl;
	cout << "\t init_keyframe_trans_thres = " << init_keyframe_trans_thres << endl;
	cout << "\t init_keyframe_rot_thres - = " << init_keyframe_rot_thres << endl;
	cout << "------------------------------------------" << endl;
	return 0;
}

void VelodyneOdom::_initialize(){
	_approximate_voxel_filter.setLeafSize (_params.approximate_voxel_leaf_size[0],
			_params.approximate_voxel_leaf_size[1],
			_params.approximate_voxel_leaf_size[2]);

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
	// Setting max number of registration iterations.
	_ndt.setMaximumIterations (_params.ndt_max_iter);
	_batch_ndt.setMaximumIterations (_params.batch_ndt_max_iter);

	_batch_optimizing = false;

	_map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_keyframes.clear();
	_keyframe_poses.clear();
	//_keyframes = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//_keyframe_pose.setIdentity();
	_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_filtered_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_aligned_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	_params.print();
}

VelodyneOdom::VelodyneOdom(){
	_initialize();
}

VelodyneOdom::VelodyneOdom(const VelodyneOdomParams &params){
	_params = params;
	_initialize();
}

double VelodyneOdom::_pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2){
	Eigen::Matrix4d dpose = pose1.inverse() * pose2;
	Eigen::Vector3d rpy   = utils::trans::dcm2rpy(dpose.topLeftCorner<3, 3>()).cwiseAbs();
	return	_params.trans_offset_weight * dpose.topRightCorner<3, 1>().norm() +
		_params.rot_offset_weight[0] * rpy(0) +
		_params.rot_offset_weight[1] * rpy(1) +
		_params.rot_offset_weight[2] * rpy(2);

}

bool VelodyneOdom::_push_new_keyframe_required(const Eigen::Matrix4d &curr_pose, int key_frame_ind){
	ASSERT(key_frame_ind < (int)_keyframes.size(), "'key_frame_ind < _keyframes.size()' should hold ");

	return _pose_distance(curr_pose, _keyframe_poses[key_frame_ind]) >=
		_params.init_keyframe_trans_thres * _params.trans_offset_weight +
		_params.init_keyframe_rot_thres * (_params.rot_offset_weight[0] +
				_params.rot_offset_weight[1] +
				_params.rot_offset_weight[2]);
}

int VelodyneOdom::_find_closest_keyframe(const Eigen::Matrix4d &curr_pose){
	double best_keyframe_dist = 9999999;
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
	return 0;
}

int VelodyneOdom::_batch_optimize(int start_keyframe_ind, int end_keyframes_ind){
	return 0;
}

int VelodyneOdom::_batch_optimize(const vector<int> &keyframe_indices){
	return 0;
}

int VelodyneOdom::_cancel_batch_optimization(bool undo_changes){
	return 0;
}

int VelodyneOdom::push_pc(const sensor_msgs::PointCloud2 &pc){
	pcl::fromROSMsg(pc, *_pc);	
	// If '_map' is empty and/or '_key_frame' is not set, copy.
	if(_map->size() == 0)
		_map = _pc->makeShared();
	if(_keyframes.empty()){
		_keyframes.push_back(_pc->makeShared());
		_keyframe_poses.push_back(Eigen::Matrix4d::Identity());
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

	_approximate_voxel_filter.setInputCloud(_pc);
	_approximate_voxel_filter.filter(*_filtered_pc);

	_ndt.setInputSource(_filtered_pc);

	_curr_keyframe_ind = _find_closest_keyframe(init_pose);
	cout << "closest_keyframe_ind = " << _curr_keyframe_ind << endl;

	_ndt.setInputTarget(_keyframes[_curr_keyframe_ind]);

	_ndt.align(*_aligned_pc, init_pose.cast<float>());

	//std::cout << "Normal Distributions Transform has converged:" << _ndt.hasConverged ()
	//		  << " score: " << _ndt.getFitnessScore () << std::endl;

	_pc_pose = _ndt.getFinalTransformation().cast<double>();

	if(_ndt.hasConverged()){
		if(_push_new_keyframe_required(_pc_pose, _curr_keyframe_ind)){
				pcl::transformPointCloud(*_pc, *_aligned_pc, _ndt.getFinalTransformation());
				*_map += *_aligned_pc;
				_keyframes.push_back(_aligned_pc->makeShared());
				_keyframe_poses.push_back(_pc_pose);
				/*
				   _approximate_voxel_filter.setInputCloud(_map);
				   _approximate_voxel_filter.filter (*_map);
				   _keyframe_pose = _pc_pose;
				 *_keyframe_pc  = *_aligned_pc;
				 }
				 */
		}
	}
	return _ndt.hasConverged() ? 0 : -1;
}

