/*
	See the header file for detailed explanations
	of the below functions.
*/

#include "range_based_tunnel_localizer.hh"

bool RangeBasedTunnelLocalizer::reset(){
	_pc->points.clear();
	_num_laser_pushes = 0;
	_num_rgbd_pushes  = 0;
	_pose *= 0;
	_fim *= 0;
	_fitness_score = 0;
	return true;
}

bool RangeBasedTunnelLocalizer::set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &map){
	_icp.setInputTarget(map);
	return true;
}

bool RangeBasedTunnelLocalizer::push_laser_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::LaserScan &data, const vector<char> &mask){
	// ### This still does not incorporate the color information
	assert(mask.size() == 0 || data.ranges.size() == mask.size());

	// Reserve required space to prevent repetitive memory allocation
	_pc->points.reserve(_pc->points.size() + data.ranges.size());

	Eigen::Vector4d pt;
	double th = data.angle_min;
	for(int i = 0 ; i < (int)data.ranges.size() ; i++, th += data.angle_increment){
		if(mask.size() != 0 || mask[i] == false)
			continue;
		utils::polar2euclidean(data.ranges[i], th, pt(0), pt(1));
		pt(2) = pt(3) = 0;
		pt = rel_pose * pt;
		_pc->points.push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
	}

	// Accumulate information due to the laser scanner onto the _fim matrix.
	// Each additional information is in the body frame. In the 'get_covariance(...)'
	// function (points have to be registered) this is projected onto the world 
	// frame.
	Eigen::Matrix3d fim_xyz = Eigen::Matrix3d::Zero();			// Fisher information for x, y, z coords.
	Eigen::Matrix3d dcm		= rel_pose.topLeftCorner<3, 3>();	// Rotation matrix
	Eigen::Matrix3d fim_xyp;									// Fisher information for x, y, yaw
	double fi_p = 0;											// Fisher information for yaw only (neglecting correlative information)
	
	utils::get_fim(data, mask, fim_xyp);
	fim_xyz.topLeftCorner<2, 2>() = fim_xyp.topLeftCorner<2, 2>();
	fim_xyz = dcm.transpose() * fim_xyz * dcm;
	
	fi_p = fim_xyp(2, 2) * dcm(2, 2);

	_fim.block<3, 3>(0, 0) += fim_xyz;
	_fim(3, 3) += fi_p;

	_num_laser_pushes++;
	return true;
}

bool RangeBasedTunnelLocalizer::push_rgbd_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::PointCloud2 &data, const vector<char> &mask){
	// ### This still does not incorporate the color information
	assert(mask.size() == 0 || data.data.size() == mask.size());

	// Reserve required space to prevent repetitive memory allocation
	_pc->points.reserve(_pc->points.size() + data.data.size());

	Eigen::Vector4d pt;
	
	ROS_WARN("\"push_rgbd_data(...)\" is not implemented yet!");
	for(int i = 0 ; i < (int)data.data.size() ; i++){
		if(mask.size() != 0 || mask[i] == false)
			continue;
		// ### To be implemented!
	}

	_num_rgbd_pushes++;
	return true;
}

bool RangeBasedTunnelLocalizer::estimate_pose(const Eigen::Matrix4d &init_pose){
	// ### This part might need some optimization.
	// If this class is used with PF, then according to previous
	// transformations, new transformations can be concatenated 
	// easily and efficiently without copying the whole data
	static pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed;
	pcl::copyPointCloud(*_pc, *pc_transformed);
	pcl::transformPointCloud(*_pc, *pc_transformed, init_pose);

	_icp.setInputSource(pc_transformed);

	_icp.align(*_pc_aligned);
	
	_pose = _icp.getFinalTransformation().cast<double>();
	_fitness_score = _icp.getFitnessScore();

	return _icp.hasConverged();
}

bool RangeBasedTunnelLocalizer::get_pose(Eigen::Matrix3d &dcm , Eigen::Vector3d &pos){
	dcm = _pose.topLeftCorner<3, 3>().cast<double>();
	pos = _pose.topRightCorner<3, 1>().cast<double>();
	return true;
}

bool RangeBasedTunnelLocalizer::get_registered_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc){	
	pcl::copyPointCloud(*_pc_aligned, *pc);
	return true;
}

bool RangeBasedTunnelLocalizer::get_covariance(Eigen::Matrix6d &cov){
	// ### I have to find a way to project uncertainties in orientation
	// to other frame sets.
	// ### I might have to fix some elements before inverting
	for(int i = 0 ; i < 6 ; i++){
		if(fabs(_fim(i, i)) < 0.01)
			_fim(i, i) = 0.01;
	}
	cov = _fim.inverse();
	return true;
}

double RangeBasedTunnelLocalizer::get_fitness_score(){
	return _fitness_score;
}
