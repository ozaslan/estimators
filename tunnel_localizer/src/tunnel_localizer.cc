#include "tunnel_localizer.hh"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define DEBUG 0
#if DEBUG == 1
#define DEBUG_MSG PRINT_FLF
#else
#define DEBUG_MSG 
#endif

bool TunnelLocalizer::push_lidar_data(const LaserProc &laser_proc, bool clean_start){
	// 'push_lidar_data(...)' is responsible for checking a clean
	// start is required or not. It searches for all of the previously 
	// pushed lidar data to see if the current data is from a previously 
	// pushed lidar data. If yes, and also if the time stamp is new,
	// a clean start is made. If the stamp is the same too, return 'false'.
	// If the source is completely different from the provious sources,
	// then push the data.
	DEBUG_MSG; 
	const LidarCalibParams& params = laser_proc.get_calib_params();
	ros::Time stamp = laser_proc.get_timestamp();
	for(int i = 0 ; i < (int)_lidar_ids.size() ; i++ ){
		if(_lidar_ids[i] == params.unique_id){
			_lidar_data_available[i] = true;
			if(_lidar_stamps[i].first  == stamp.sec &&
					_lidar_stamps[i].second == stamp.nsec)
				return false;
			else {
				clean_start = true;
				break;
			}
		}
	}

	if(clean_start == true){
		_lidar_ids.clear();
		_lidar_stamps.clear();
		_lidar_data_available.clear();
	}

	_lidar_ids.push_back(params.unique_id);
	_lidar_stamps.push_back(std::make_pair<double, double>(
				stamp.sec, stamp.nsec));
	_lidar_data_available.push_back(true);

	_rbtl.push_laser_data(laser_proc, clean_start);

	DEBUG_MSG;
	return true;
}

bool TunnelLocalizer::push_lidar_data(const sensor_msgs::LaserScan &data, const LidarCalibParams  &params, bool clean_start){
	// 'push_lidar_data(...)' is responsible for checking a clean
	// start is required or not. It searches for all of the previously 
	// pushed lidar data to see if the current data is from a previously 
	// pushed lidar data. If yes, and also if the time stamp is new,
	// a clean start is made. If the stamp is the same too, return 'false'.
	// If the source is completely different from the provious sources,
	// then push the data.
	DEBUG_MSG;
	for(int i = 0 ; i < (int)_lidar_ids.size() ; i++ ){
		if(_lidar_ids[i] == params.unique_id){
			_lidar_data_available[i] = true;
			if(_lidar_stamps[i].first  == data.header.stamp.sec &&
					_lidar_stamps[i].second == data.header.stamp.nsec)
				return false;
			else {
				clean_start = true;
				break;

			}
		}
	}

	if(clean_start == true){
		_lidar_ids.clear();
		_lidar_stamps.clear();
		_lidar_data_available.clear();
	}

	_lidar_ids.push_back(params.unique_id);
	_lidar_stamps.push_back(std::make_pair<double, double>(
				data.header.stamp.sec, 
				data.header.stamp.nsec));
	_lidar_data_available.push_back(true);

	// Mask the regions to be omitted (dead_regions)	
	int num_clusters = 0;
	static vector<char> mask;
	mask.resize(data.ranges.size());
	if(params.dead_regions.size() != 0){
		num_clusters = 1;
		std::fill(mask.begin(), mask.end(), 1);
	}

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

	// Use utils to cluster scan	
	utils::laser::cluster_laser_scan(data, mask, num_clusters, params.min_range, params.max_range);
	// ### (to be done) manually assign new clusters to upward and downward rays

	for(int i = 1 ; i <= num_clusters ; i++)
		_rbtl.push_laser_data(params.relative_pose, data, mask, i, i == 1 && clean_start);

	// ### update relative pose for upwards and downwards rays. 
	// push_laser_data(...)

	DEBUG_MSG;
	return true;
}

bool TunnelLocalizer::push_rgbd_data(const sensor_msgs::PointCloud2 &rdgb, const RGBDCalibParams &params, bool clean_start){
	//### To be implemented
	return true;
}

// Note to share with Dr. Kumar : TunnelLocalizer is a blocking estimator.
// When N number of similar sensors are used, estimation is available with
// the frequency of the slowest of sensors. Blocking estimators cannot
// do estimation unless data from 'all sources' are available. Such estimators
// are designed to work with partial information.


bool TunnelLocalizer::push_camera_data(const sensor_msgs::Image &data, const CameraCalibParams &params){	
	// 'push_camera_data(...)' is responsible for keeping track
	// of registered cameras. Everytime a new camera calibration
	// data arrives, it is registered to VisionBasedTunnelLocalizer.
	// This causes a reseting VisionBasedTunnelLocalizer, this will
	// interrupt the estimation.

	//checking a clean
	// start is required or not. It searches for all of the previously 
	// pushed lidar data to see if the current data is from a previously 
	// pushed lidar data. If yes, and also if the time stamp is new,
	// a clean start is made. If the stamp is the same too, it returns 'false'.
	// If the source is completely different from the provious sources,
	// then it pushes the data.

	//cout << "data.[width, height] = [" << data.width << ", " << data.height << "]" << endl;

	DEBUG_MSG;
	bool is_new_camera = true;
	cv_bridge::CvImagePtr image_msg;
	for(int i = 0 ; i < (int)_camera_ids.size() ; i++ ){
		if(_camera_ids[i] == params.unique_id){
			if(_camera_stamps[i].first  == data.header.stamp.sec &&
					_camera_stamps[i].second == data.header.stamp.nsec)
				return false;
			else {
				image_msg = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
				if(image_msg->image.channels() == 3)
					cv::cvtColor(image_msg->image, _frames[i], CV_BGR2GRAY);
				else
					image_msg->image.copyTo(_frames[i]);
				_camera_stamps[i].first  = data.header.stamp.sec;
				_camera_stamps[i].second = data.header.stamp.nsec;
				_camera_data_available[i] = true;
				is_new_camera = false;
				break;
			}
		}
	}

	if(is_new_camera == true){
		_camera_ids.push_back(params.unique_id);
		_camera_stamps.push_back(std::make_pair<double, double>(
					data.header.stamp.sec, 
					data.header.stamp.nsec));
		_camera_data_available.push_back(true);
		image_msg = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
		if(image_msg->image.channels() == 3){
			_frames.push_back(cv::Mat(0, 0, CV_8U));
			cv::cvtColor(image_msg->image, _frames.back(), CV_BGR2GRAY);
		} else
			_frames.push_back(image_msg->image);
		_cam_params.push_back(params);
		_vbtl.register_camera_params(_cam_params);
	}

	bool all_frames_present = true;
	for(int i = 0 ; i < (int)_camera_data_available.size() ; i++)
		all_frames_present = all_frames_present && _camera_data_available[i];

	if(all_frames_present == false)
		return false;
	else {
		Eigen::Matrix4d pose;
		_rbtl.get_pose(pose);
		_vbtl.push_camera_data(_frames, pose);
		// Invalidate all the camera data.
		std::fill(_camera_data_available.begin(), 
				_camera_data_available.end(), false);
		return true;
	}
	DEBUG_MSG;
}

bool TunnelLocalizer::estimate_pose(const Eigen::Matrix4d &init_pose, double heading){
	DEBUG_MSG;
	// ### I have to have options for particle filter,
	// freedoms to update, initial pose etc...
	_rbtl.estimate_pose(init_pose, heading);
	_vbtl.estimate_displacement(_x_disp);
	/*
	bool all_frames_present = true;
	for(int i = 0 ; i < (int)_camera_data_available.size() ; i++)
		all_frames_present = all_frames_present && _camera_data_available[i];

	if(all_frames_present == false){

	} else {
		Eigen::Matrix4d pose;
		_rbtl.get_pose(pose);
		_vbtl.push_camera_data(_frames, pose);

		_vbtl.estimate_displacement(_x_disp);
		// Invalidate all the camera data.
		std::fill(_camera_data_available.begin(), 
				_camera_data_available.end(), false);
	}
	*/
	DEBUG_MSG;
	return true;
}

bool TunnelLocalizer::reset(){
	//_rbtl.reset();
	//_vbtl.reset();
	return true;
}

bool TunnelLocalizer::set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &grid_map){
	DEBUG_MSG;
	_map = grid_map;
	_rbtl.set_map(_map);
	_vbtl.set_map(_map);
	DEBUG_MSG;
	return true;
}

bool TunnelLocalizer::get_pose(Eigen::Matrix4d &pose){
	DEBUG_MSG;
	_rbtl.get_pose(pose);

	double x_disp;
	_vbtl.get_displacement(x_disp);
	pose(0, 3) += x_disp;
	static double total_x_disp = 0;
	//cout << "x_disp : " << x_disp << endl;
	total_x_disp += x_disp;
	cout << "total_x_disp = " << total_x_disp << endl;
	//cout << "pose = " << pose << endl;
	DEBUG_MSG;
	return true;
}

bool TunnelLocalizer::get_covariance(Eigen::Matrix6d &cov){
	DEBUG_MSG;
	_rbtl.get_covariance(cov, false);
	double x_var;
	_vbtl.get_variance(x_var);
	//cov(0, 0) += x_var;
	// ### from VisionBasedTunnelLocalizer too?
	DEBUG_MSG;
	return true;
}


bool TunnelLocalizer::get_registered_lidar_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud ){
	DEBUG_MSG; 
	_rbtl.get_registered_pointcloud(cloud);
	DEBUG_MSG;
	return true;
}

bool TunnelLocalizer::get_range_map_correspondences(vector<pcl::PointXYZ> &range_pts, vector<pcl::PointXYZ> &map_pts){
	DEBUG_MSG;
	return _rbtl.get_correspondences(range_pts, map_pts);
}

bool TunnelLocalizer::get_back_projected_features(vector<pcl::PointXYZ> &tails, vector<pcl::PointXYZ> &tips){
	DEBUG_MSG;
	_vbtl.get_back_projected_flow_vectors(tails, tips);
	DEBUG_MSG; 
	return true;
}

bool TunnelLocalizer::plot_tracked_features(vector<cv::Mat> &images, bool plot_flow, bool plot_feats){
	DEBUG_MSG;
	if(_frames.size() == 0 || _frames[0].rows * _frames[0].cols == 0){
		DEBUG_MSG;
		return false;
	}

	_vbtl.plot_flows(_frames, plot_flow, plot_feats);
	images.resize(_frames.size());
	for(int i = 0 ; i < (int)_frames.size() ; i++)
		images[i] = _frames[i];
	DEBUG_MSG;
	return true;
}
