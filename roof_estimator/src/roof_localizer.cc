#include "roof_localizer.hh"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

bool RoofLocalizer::push_lidar_data(const sensor_msgs::LaserScan &data, const LidarCalibParams  &params, bool clean_start){
	// 'push_lidar_data(...)' is responsible for checking a clean
	// start is required or not. It searches for all of the previously 
	// pushed lidar data to see if the current data is from a previously 
	// pushed lidar data. If yes, and also if the time stamp is new,
	// a clean start is made. If the stamp is the same too, return 'false'.
	// If the source is completely different from the provious sources,
	// then push the data.
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
	utils::cluster_laser_scan(data, mask, num_clusters, params.min_range, params.max_range);
	// ### (to be done) manually assign new clusters to upward and downward rays

	for(int i = 1 ; i <= num_clusters ; i++)
		_rbtl.push_laser_data(params.relative_pose, data, mask, i, i == 1 && clean_start);

	// ### update relative pose for upwards and downwards rays. 
	// push_laser_data(...)

	return true;
}

bool RoofLocalizer::push_rgbd_data(const sensor_msgs::PointCloud2 &rdgb, const RGBDCalibParams &params, bool clean_start){
	//### To be implemented
	return true;
}

// Note to share with Dr. Kumar : RoofLocalizer is a blocking estimator.
// When N number of similar sensors are used, estimation is available with
// the frequency of the slowest of sensors. Blocking estimators cannot
// do estimation unless data from 'all sources' are available. Such estimators
// are designed to work with partial information.


bool RoofLocalizer::push_camera_data(const sensor_msgs::Image &data, const CameraCalibParams &params){	
	// 'push_camera_data(...)' is responsible for keeping track
	// of registered cameras. Everytime a new camera calibration
	// data arrives, it is registered to VisionBasedRoofLocalizer.
	// This causes a reseting VisionBasedRoofLocalizer, this will
	// interrupt the estimation.

	//checking a clean
	// start is required or not. It searches for all of the previously 
	// pushed lidar data to see if the current data is from a previously 
	// pushed lidar data. If yes, and also if the time stamp is new,
	// a clean start is made. If the stamp is the same too, return 'false'.
	// If the source is completely different from the provious sources,
	// then push the data.

	cout << "data.[width, height] = [" << data.width << ", " << data.height << "]" << endl;

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

	cout << "all_frames_present = " << all_frames_present << endl;

	if(all_frames_present == false)
		return false;
	else {
		Eigen::Matrix4d pose;
		_rbtl.get_pose(pose);
		cout << "_frames.size() = " << _frames.size() << endl;
		_vbtl.push_camera_data(_frames, pose);
		// Invalidate all the camera data.
		std::fill(_camera_data_available.begin(), 
				  _camera_data_available.end(), false);

		return true;
	}
}

bool RoofLocalizer::estimate_pose(const Eigen::Matrix4d &init_pose, const octomap::OcTree &octomap){
	// ### I have to have options for particle filter,
	// freedoms to update, initiali pose etc...
	_rbtl.estimate_pose(init_pose, octomap);

	// ### didn't like this
	_vbtl.estimate_displacement(_x_disp);
	_x_pos += _x_disp;
	cout << "_x_disp = " << _x_disp << endl;

	return true;
}

bool RoofLocalizer::reset(){
	//_rbtl.reset();
	//_vbtl.reset();
	return true;
}

bool RoofLocalizer::get_pose(Eigen::Matrix4d &pose){
	Eigen::Matrix3d dcm;
	Eigen::Vector3d pos;
	pose = Eigen::Matrix4d::Identity();
	_rbtl.get_pose(pose);
	//pose.topLeftCorner(3, 3)  = dcm;
	//pose.topRightCorner(3, 1) = pos;
	//pose(3, 3) = 1;
	// ### from VisionBasedRoofLocalizer?
	pose(0, 3) = _x_pos;
	return true;
}

bool RoofLocalizer::get_covariance(Eigen::Matrix6d &cov){
	_rbtl.get_covariance(cov);
	// ### from VisionBasedRoofLocalizer too?
	return true;
}


bool RoofLocalizer::get_registered_lidar_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud ){
	_rbtl.get_registered_pointcloud(cloud);
	return true;
}

bool RoofLocalizer::get_range_map_correspondences(vector<pcl::PointXYZ> &range_pts, vector<pcl::PointXYZ> &map_pts){
	return _rbtl.get_correspondences(range_pts, map_pts);
}

bool RoofLocalizer::get_back_projected_features(vector<pcl::PointXYZ> &tails, vector<pcl::PointXYZ> &tips){
	_vbtl.get_back_projected_flow_vectors(tails, tips);
	return true;
}

bool RoofLocalizer::plot_tracked_features(cv::Mat &image, bool plot_flow, bool plot_feats){
	if(_frames[0].rows * _frames[0].cols == 0){
		cout << "returning false"  << endl << endl;
		return false;
	}
	_vbtl.plot_flows(_frames, true, plot_flow, plot_feats);
	image = _frames[0];
	return true;
}
