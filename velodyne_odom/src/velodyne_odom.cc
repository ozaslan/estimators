#include "velodyne_odom.hh"
#include "utils.hh"

void VelodyneOdom::_initialize(){
	_approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	_ndt.setTransformationEpsilon (0.05);
	// Setting maximum step size for More-Thuente line search.
	_ndt.setStepSize (0.1);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	_ndt.setResolution (1.0);
	// Setting max number of registration iterations.
	_ndt.setMaximumIterations (15);

	_map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_keyframe_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_keyframe_pose.setIdentity();
	_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_filtered_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	_aligned_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

}

VelodyneOdom::VelodyneOdom(){
	_initialize();
}

int VelodyneOdom::push_pc(const sensor_msgs::PointCloud2 &pc){
	pcl::fromROSMsg(pc, *_pc);	
	// If '_map' is empty and/or '_key_frame' is not set, copy.
	if(_map->size() == 0)
		_map = _pc->makeShared();
	if(_keyframe_pc->size() == 0){
		_keyframe_pc = _pc->makeShared();
		_keyframe_pose = Eigen::Matrix4d::Identity();
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

	_approximate_voxel_filter.setInputCloud (_pc);
	_approximate_voxel_filter.filter (*_filtered_pc);

	_ndt.setInputSource (_filtered_pc);

	_ndt.setInputTarget (_keyframe_pc);

	_ndt.align (*_aligned_pc, init_pose.cast<float>());

	//std::cout << "Normal Distributions Transform has converged:" << _ndt.hasConverged ()
	//		  << " score: " << _ndt.getFitnessScore () << std::endl;

	_pc_pose = _ndt.getFinalTransformation().cast<double>();

	if(_ndt.hasConverged()){
		Eigen::Matrix4d dpose = _pc_pose.inverse() * _keyframe_pose;
		Eigen::Vector3d rpy   = utils::trans::dcm2rpy(dpose.topLeftCorner<3, 3>());
		double angle_th = DEG2RAD(5);
		double trans_th = 0.23;
		if(	fabs(rpy(0)) > angle_th || 
			fabs(rpy(1)) > angle_th || 
			fabs(rpy(2)) > angle_th ||
			fabs(dpose(0, 3)) > trans_th ||
			fabs(dpose(1, 3)) > trans_th ||
			fabs(dpose(2, 3)) > trans_th){
			pcl::transformPointCloud(*_pc, *_aligned_pc, _ndt.getFinalTransformation());
			*_map += *_aligned_pc;
			_approximate_voxel_filter.setInputCloud(_map);
			_approximate_voxel_filter.filter (*_map);

			_keyframe_pose = _pc_pose;
			*_keyframe_pc  = *_aligned_pc;
		}
	}


	//cout << "PC POSE : " << _pc_pose << endl;

	return _ndt.hasConverged() ? 0 : -1;
}










