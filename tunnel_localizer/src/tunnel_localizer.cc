#include "tunnel_localizer.hh"

bool TunnelLocalizer::push_lidar_data(const sensor_msgs::LaserScan &data, const LidarCalibParams  &params){
	// -- cluster scan data
	// -- push clusters one-by-one

	//cout << "-- P1" << endl;
	// mask the regions to be omitted (dead_regions)	
	int num_clusters = 0;
	static vector<char> mask;
	mask.resize(data.ranges.size());
	if(params.dead_regions.size() != 0){
		num_clusters = 1;
		std::fill(mask.begin(), mask.end(), 1);
	}

	//for(int i = 0 ; i < params.dead_regions.size() ; i++)
	//	cout << "params.dead_regions[" << i << "] = " << params.dead_regions[i] << endl;

	//cout << "num_clusters : " << num_clusters << endl;

	//cout << "-- P2" << endl;
	for(int i = 0 ; i < (int)params.dead_regions.size() ; i+=2){
		double th_begin = params.dead_regions[i];
		double th_end   = params.dead_regions[i+1];
		int idx_begin = std::max(0.0, 
								 round((th_begin - data.angle_min) / data.angle_increment));
		int idx_end   = std::min(data.ranges.size() - 1.0, 
								 round((th_end   - data.angle_min) / data.angle_increment));
		for(int j = idx_begin ; j <= idx_end ; j++)
			mask[j] = 0;
	}

	//for(int i = 0 ; i < mask.size() ; i++)
	//	cout << (int)mask[i];

	//cout << "-- P3" << endl;
	// use utils to cluster scan	


	utils::cluster_laser_scan(data, mask, num_clusters, params.min_range, params.max_range);
	// ### (to be done) manually assign new clusters to upward and downward rays

	//for(int i = 0 ; i < mask.size() ; i++)
	//	cout << (int)mask[i];

	//cout << "num_cluster : " << num_clusters << endl;
	//for(int i = 0 ; i < mask.size() ; i++)
	//	cout << (int)mask[i];

	//cout << "-- P4" << endl;
	//cout << "num_clusters : " << num_clusters << endl;
	for(int i = 1 ; i <= num_clusters ; i++)
		_rbtl.push_laser_data(params.relative_pose, data, mask, i);

	//cout << "-- P5" << endl;
	// ### update relative pose for upwards and downwards rays. 
	// push_laser_data(...)

	return true;
}

bool TunnelLocalizer::push_rgbd_data(const sensor_msgs::PointCloud2 &rdgb, const RGBDCalibParams &params){
	//### To be implemented
	return true;
}

bool TunnelLocalizer::push_camera_data(const sensor_msgs::Image &frame, const CameraCalibParams &params){	
	//### To be implemented
	return true;
}

bool TunnelLocalizer::estimate_pose(const Eigen::Matrix4d &init_pose){
	// ### I have to have options for particle filter,
	// freedoms to update, initiali pose etc...
	return _rbtl.estimate_pose(init_pose);
}

bool TunnelLocalizer::reset(){
	_rbtl.reset();
	// ### reset camera and rgbd as well
	return true;
}

bool TunnelLocalizer::set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &grid_map){
	_map = grid_map;
	_rbtl.set_map(_map);
	// ### set map for VisionBasedTunnelLocalizer
	return true;
}

bool TunnelLocalizer::get_pose(Eigen::Matrix4d &pose){
	Eigen::Matrix3d dcm;
	Eigen::Vector3d pos;
	pose = Eigen::Matrix4d::Zero();
	_rbtl.get_pose(dcm, pos);
	pose.topLeftCorner(3, 3)  = dcm;
	pose.topRightCorner(3, 1) = pos;
	pose(3, 3) = 1;
	return true;
}

bool TunnelLocalizer::get_covariance(Eigen::Matrix6d &cov){
	_rbtl.get_covariance(cov);
	return true;
}


bool TunnelLocalizer::get_registered_lidar_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud ){
	_rbtl.get_registered_pointcloud(cloud);
	return true;
}

bool TunnelLocalizer::get_range_map_correspondences(vector<pcl::PointXYZ> &range_pts, vector<pcl::PointXYZ> &map_pts){
	return _rbtl.get_sensor_map_correspondences(range_pts, map_pts);
}
