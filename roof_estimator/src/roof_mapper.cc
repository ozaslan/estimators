#include "roof_mapper.hh"

/*
	This class gets laser, RGBD and camera data as
	input and builds a 3D octomap of the environment and
	a sparse image based map concurrently. All data
	is provided with its corresponding pose which is assumed 
	to be GT or accurate enough. In other words, the provided 
	poses are not tuned further. The API user gives
	the pose estimate and the corresponding calibration
	parameters including the relative extrinsic parameters after 
	merging which the sensor pose is obtained. This provides 
	a simpler interface for the user. According	to the 
	markers/definition in the parameter set, sensor data is 
	partitioned into clusters in the simplest case which may 
	consist of 'valid' and 'neglect' classes. After filtering,
	range data is integrated into an octomap. A separate image
	based sparse maps is build concurrently from image features 
	persistent over time. Both type of mapping lack loop detection-
	map correction feature.
*/

RoofMapper::RoofMapper(){
	//_rbrm;
	//_vbrm;
	//_lidar_ids;
	//_camera_ids;
}

bool RoofMapper::register_lidar_data (const Eigen::Matrix4d &pose, const sensor_msgs::LaserScan &scan  , const LidarCalibParams  &params, bool clean_start){
	static Eigen::Matrix4d sensor_pose;
	static vector<char> mask;
	
	// ### clean_start = true is not implemented.
	
	bool new_device = true;
	for(int i = 0 ; i < (int)_lidar_ids.size() ; i++){
		if(_lidar_ids[i] == params.unique_id){
			new_device = false;
			break;
		}
	}

	if(new_device == true)
		_lidar_ids.push_back(params.unique_id);
	
	//cout << "HERE 1" << endl;
	//cout << scan.ranges.size() << endl;
	// Mask the regions to be omitted (dead_regions)	
	mask.resize(scan.ranges.size());
	std::fill(mask.begin(), mask.end(), 1);

	for(int i = 0 ; i < (int)params.dead_regions.size() ; i+=2){
		double th_begin = params.dead_regions[i];
		double th_end   = params.dead_regions[i+1];
		int idx_begin   = std::max(0.0, 
				round((th_begin - scan.angle_min) / scan.angle_increment));
		int idx_end     = std::min(scan.ranges.size() - 1.0, 
				round((th_end   - scan.angle_min) / scan.angle_increment));
		for(int j = idx_begin ; j <= idx_end ; j++)
			mask[j] = 0;
	}	

	int num_clusters;
	utils::cluster_laser_scan(scan, mask, num_clusters, params.min_range, params.max_range, 3, 0.3, 3);
	// cluster_laser_scan(...) eliminates small clusters with the expense of 
	// increasing the number of clusters. In order to integrate all range data
	// at once, we mark all non-zero mask elements to '1'.
	for(int i = 0 ; i < (int)mask.size() ; i++)
		if(mask[i] != 0)
			mask[i] = 1;

	sensor_pose = pose * params.relative_pose;
	//sensor_pose = params.relative_pose * pose;
	//cout << "fed pose to the 'roof_mapper : " << endl << pose << endl;
	//cout << "sensor relative pose = " << endl << params.relative_pose << endl;
	//cout << "sensor_pose in 'roof_mapper' = " << endl << sensor_pose << endl;
	_rbrm.register_scan(sensor_pose, scan, mask, 1, new_device ? 1.50 : 0);
	return true;
}

bool RoofMapper::register_rgbd_data  (const Eigen::Matrix4d &pose, const sensor_msgs::PointCloud2 &rdgb, const RGBDCalibParams   &params, bool clean_start){
	return true;
}

bool RoofMapper::register_camera_data(const Eigen::Matrix4d &pose, const sensor_msgs::Image &frame	   , const CameraCalibParams &params, bool clean_start){
	return true;
}

bool RoofMapper::reset(){
	_rbrm.reset();
	_vbrm.reset();
	return true;
}

const octomap::OcTree& RoofMapper::get_octomap(){
	return _rbrm.get_map();
}

const bool& get_vismap(){
	return true;
}


