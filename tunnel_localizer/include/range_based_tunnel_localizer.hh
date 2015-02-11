#ifndef __RANGE_BASED_TUNNEL_LOC_HH__
#define __RANGE_BASED_TUNNEL_LOC_HH__

#include <cmath>
#include <vector>
#include <limits>
#include <numeric>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <utils.hh>

/*	This class gives a pose estimate for the robot in a known environment. 
	A map point cloud is given as an argument. Also an initial estimate 
	of the pose is expected. The user can select DOFs to updated while others 
	are kept constant. Sensor inputs consist of laser and RGBD data. From 
	the sensor data, it constructs a single collective point cloud structure 
	and runs PCL's ICP. The output is the ICP pose and a 3D approxrimation 
	of Censi's covariance estimate. */

// ### I should better assume orientation to be represented as quadternions 
// and calculate uncertainty in that space. Also this will require me to
// return orientation in quat as well instead of dcm for the sake of 
// consistancy.


class RangeBasedTunnelLocalizer{
private:
	int _num_laser_pushes;	// Number of pushes since the last 'estimate_pose'
	int _num_rgbd_pushes;	// call with a 'clear_data' flag.
	pcl::PointCloud<pcl::PointXYZ>::Ptr _pc;						// Collective pointcloud sensor data
	pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_aligned;				// '_pc' after being aligned
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;	// We are going to use functions of this class for pose estimation
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *_octree;
	vector<pcl::PointXYZ> _matched_map_pts;
	Eigen::Matrix4d _pose;  // Pose of the robot (output of ICP registration)
	Eigen::Matrix6d _fim;	// Fisher information matrix
	double _fitness_score;	// Wellness of the fit

	bool _get_covariance(const sensor_msgs::LaserScan &data, Eigen::Matrix3d &cov);
	bool _get_covariance(const sensor_msgs::PointCloud2 &data, Eigen::Matrix6d &cov);
public:
	RangeBasedTunnelLocalizer();
	// This function resets the internal flags, counters, estimates, clears the
	// collvective point cloud. If not set explicitely this function is called 
	// after each 'estimate_pose(...)' function. 
	bool reset();
	// This function sets the internal map pointer. Changes in the referenced 
	// map do reflect to the internal copy of the map pointer. Thus this function
	// call is of contant time complexity.
	bool set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &map);
	// This adds laser points to the collective sensor pointcloud 
	// 'rel_pose' is the pose of the laser scanner in the body frame
	// 'mask' is a boolean vector where a 'false' denotes its corresponding 
	// data should be negleceted. All data is used if sizes of 'ranges' 
	// and 'mask' do not match.
	bool push_laser_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::LaserScan &data, const vector<char> &mask, char cluster_id);
	// This adds pointcloud, possibly from an rgbd sensor,
	// to the collvective sensor pointcloud.
	// 'rel_pose' is the pose of the RGBD sensor  in the body frame
	// 'mask' is a boolean vector where a 'false' denotes its corresponding 
	// data should be negleced. 'mask' should be indexed in row-major
	// order. All data is used if the sizes of 'data' and 'mask' do
	// not match.
	bool push_rgbd_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::PointCloud2 &data, const vector<char> &mask, char cluster_id);
	// This function run the PCL ICP function starting from the 
	// given initial pose. 'get_pose', 'get_covariance' and 
	// 'get_fitness_score' function are used to get results.
	// Care should be taken due to the PCL ICP's being a black
	// box. Precaution should be taken for possible drastically
	// wrong pose estimates.
	// ### Write your own ICP. Until I implement mine, I cannot
	// constrain updates to certain coordinates.
	bool estimate_pose(const Eigen::Matrix4d &init_pose);
	// This function assumes that 'estimate_pose' function is called
	// beforehand. When properly called, it returns the pose in 
	// world frame. Otherwise last pose residing from the most 
	// recent estimate is returned.
	bool get_pose(Eigen::Matrix3d &dcm , Eigen::Vector3d &pos);
	// This function, when it is called after the 'estimate_pose'
	// function, rigidly transforms the collective pointcloud to
	// the estimated pose, then returns. It does not have any
	// effect on the execution of the estimation process but
	// provides material for visualization. If a rgbd data is
	// pushed and/or laser intensities are available, pointcloud
	// will include color information as well.
	bool get_registered_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);
	// This estimates the covariance of the measurement model
	// using Censi's method using the shape of the 3D point cloud.
	// Depending on the pushed data, either 2D method or a new 3D
	// method is used. See the related paper of Censi for the 
	// former method. ### To be implemented ### If a 3D point
	// cloud is pushed, instead of lines, planes are used 
	// while estimation information of each ray  (yet point of
	// the rgbd data). This function gives higher covariances
	// if the registration fitness is low.
	bool get_covariance(Eigen::Matrix6d &cov);
	// This function, independent of the covariance estimate,
	// gives a score of wellness of the fit. Higher values 
	// mean a good fit.
	double get_fitness_score();

	bool get_sensor_map_correspondences(vector<pcl::PointXYZ> &sensor_pts, vector<pcl::PointXYZ> &map_pts);
};


#endif
