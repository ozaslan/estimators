// !!! Definition : A mapper is a process which takes
// sensor inputs with their poses  and some parameters as input, 
// and outputs a full/partial map.
// Input-1  : sensor data
// Input-2  : parameters
// Output-1 : pose estimate

#ifndef __ROOF_MAPPER_HH__
#define __ROOF_MAPPER_HH__

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

#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <calib_params.hh>
#include "range_based_roof_mapper.hh"
#include "vision_based_roof_mapper.hh"

class RoofMapper{
  private:
	// The two mappers process range or vision data to build
	// exclusive maps. 'get_octomap(...)' and 'get_vismap(...)'
	// function provide interfaces for the octomap and vision-
	// based map respectively. 'register_lidar_data(...)' and
	// 'register_rgbd_data(...)' functions feed sensor data
	// to RangeBasedRoofMapper after pre-processing.
	// 'register_camera_data(...)' and 'register_rgbd_data(...)'
	// feed sensor data to VisionBasedRoofMapper similarly.
    RangeBasedRoofMapper  _rbrm;
    VisionBasedRoofMapper _vbrm;
    
	// '_lidar_ids' and '_camera_ids' keep track of devices
	// contributed to the current maps up to present time. 
	// Introduction of a new device to the mapping process 
	// is detected by comparing the new device ID to the present 
	// list. This way, special initialization processes can be trigged.
    vector<string> _lidar_ids, _camera_ids;
	// vector<int> _num_cam_registers;
	// vector<int> _num_lidar_registers;

  public:
	RoofMapper();
    // The following functions process raw data and register to the proper lower-level mappers.
	// 'clean_start = true' forces the corresponding map to be reset first. For the case of
	// lidar registration, this will force extrusion of first instance of each lidar data.
    bool register_lidar_data (const Eigen::Matrix4d &pose, const LaserProc &laser_proc, bool clean_start = false);
    bool register_lidar_data (const Eigen::Matrix4d &pose, const sensor_msgs::LaserScan &scan  , const LidarCalibParams  &params, bool clean_start = false);
    bool register_rgbd_data  (const Eigen::Matrix4d &pose, const sensor_msgs::PointCloud2 &rdgb, const RGBDCalibParams   &params, bool clean_start = false);
    bool register_camera_data(const Eigen::Matrix4d &pose, const sensor_msgs::Image &frame	   , const CameraCalibParams &params, bool clean_start = false);
    // This function calls the reset(...) functions of RangeBasedRoofMapper and
    // VisionBasedRoofMapper. Those functions reset the private variables including 
    // the cached/collective sensor data and some internal flags.
    bool reset();
	// This function returns a constant reference of the octomap. Octomap is built
	// using the lidar and RGBD sensors. Return value of this function can be used
	// by the localizer and for visualization.
	const octomap::OcTree& get_octomap();
	// This function returns a constant reference of the vismap (vision-based map). 
	// Vismap is built using the camera and RGBD sensors. Return value of this function 
	// can be used by the localizer and for visualization.
	const bool& get_vismap();

	const octomap::Pointcloud& get_last_range_registration(){ return _rbrm.get_last_registration();}
};


#endif


