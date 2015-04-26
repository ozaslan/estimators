#ifndef __RANGE_BASED_ROOF_LOC_HH__
#define __RANGE_BASED_ROOF_LOC_HH__

#include <cmath>
#include <vector>
#include <limits>
#include <numeric>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include<octomap/OcTree.h>

#include <utils.hh>

/*	This class provides pose estimate of the robot in a known environment.
	The class requires the map as point cloud, an initial estimate,
	and point cloud or scan data from sensors. There is no soft bound
	on the total number of sensors that can be integrated, however performance 
	may degrade with excess amount of data. For aligning the sensor data
	with the map, 3DOF ICP is used with y-z-yaw being the free DOFs.
	It also provides an uncertainy measure defined as in Censi's ICRA 2009 
	paper.

	This implementation is suitable for using with particle filters as well.
	The user is supposed to push sensor data once and with different initial
	poses carry the estimation. After each iteration pose, covariance and 
	fitness of ICP can be fetched to vote for each particle in the wrapper
	class.
*/

// ### I should better assume orientation to be represented as quadternions 
// and calculate uncertainty in that space. Also this will require me to
// return orientation in quat as well instead of dcm for the sake of 
// consistancy.

// ### Be sure that IMU orientation is applied correctly to PC.

class RangeBasedRoofLocalizer{
private:
	int _num_laser_pushes;	// Number of pushes since the last memory clean-up
	int _num_rgbd_pushes;	// (the same as above)
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;			// Collective pointcloud sensor data
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_aligned;	// '_pc' after being aligned
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *_octree;
	vector<pcl::PointXYZ> _matched_map_pts;
	Eigen::Matrix4d _pose;		// Pose of the robot (output of ICP registration given as homo. SE(3))
	Eigen::Matrix6d _fim;		// Fisher information matrix
	double _fitness_scores[4];	// average of large residuals (see the ### ??? function for detailed explanation)
	double _max_iters;			// maximum iterations of the ICP algorithm.
	double _xyz_tol;				// translational tolerance value if updates of ICP is lower than which, iteration is stopped.
	double _yaw_tol;			// same as '_yz_tol' for yaw.
	// ### _skip_rays;			// number of rays to skip in ICP for performance improvement.

	bool _get_covariance(const sensor_msgs::LaserScan &data, Eigen::Matrix3d &cov);
	bool _get_covariance(const sensor_msgs::PointCloud2 &data, Eigen::Matrix6d &cov);
	
	// This function resets the internal flags, counters, clears the collvective 
	// point cloud. This is called when 'push_X_data(...)' is called with 'reset' flag.
	bool _reset();
public:
	RangeBasedRoofLocalizer(int max_iter = 23, double xyz_tol = 0.01, double yaw_tol = 0.01);
	// This adds laser points to the collective sensor pointcloud.
	// 'rel_pose' is the pose of the laser scanner in the body frame
	// 'mask' is a char vector where '0' denotes that the corresponding 
	// data should be negleceted. All data is used if sizes of 'ranges' 
	// and 'mask' do not match. Otherwise only ranges where 'mask[i] == 
	// cluster_id' are processed. If 'clean_start' is set, '_reset()'
	// is called before every other operation.
	bool push_laser_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::LaserScan &data, const vector<char> &mask, char cluster_id, bool clean_start = false);
	// This adds pointcloud, possibly from an rgbd sensor, to the 
	// collvective sensor pointcloud. 'rel_pose' is the pose of 
	// the RGBD sensor in the body frame. 'mask' is a char vector 
	// where '0' denotes that the corresponding data should be 
	// negleced. 'mask' should be indexed in row-major order. All data 
	// is used if the sizes of 'data' and 'mask' do not match. Otherwise
	// only ranges where 'mask[i] == cluster_id' are processed.
	// If 'clean_start' is set, '_reset()' is called before every 
	// other operation.
	bool push_rgbd_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::PointCloud2 &data, const vector<char> &mask, char cluster_id, bool clean_start = false);
	// This function runs the 3D ICP algorithm starting from the 
	// given initial pose. 'get_pose', 'get_covariance' and 
	// 'get_fitness_score' functions can be used to get the results.
	// In this implementation only y-z-yaw DOFs are updated. This
	// function s returns the number of iterations.
	int estimate_pose(const Eigen::Matrix4d &init_pose, const octomap::OcTree &octomap);
	// This function assumes that 'estimate_pose' function is called
	// beforehand. When properly called, it returns the pose in 
	// world frame. Otherwise last pose residing from the most 
	// recent estimate is returned.
	bool get_pose(Eigen::Matrix4d &pose);
	// This function, when it is called after the 'estimate_pose'
	// function, it returns the rigidly transforms the collective 
	// pointcloud (obtained while running ICP) the estimated pose. 
	// It does not have any effect on the execution of the estimation 
	// process but provides material for visualization. ### If a rgbd 
	// data is pushed and/or laser intensities are available, pointcloud
	// will include color information as well.
	bool get_registered_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);
	// This estimates the covariance of the measurement model
	// with Censi's method using the shape of the 3D point cloud.
	// Depending on the pushed data, either 2D method or a new 3D
	// method is used. See the related paper of Censi 2009 ICRA for 
	// the former method. ### To be implemented ### If a 3D point
	// cloud is pushed, instead of lines, planes are used 
	// in information gain of each ray. If 'apply_fitness_result = true'
	// covariance is modified to reflect the effect of '_fitness_scores'.
	// This always increases the covariance and gives are more 
	// conservative estimate. 
	bool get_covariance(Eigen::Matrix6d &cov, bool apply_fitness_result = false);
	// This function, independent of the covariance estimate,
	// gives a score of wellness of the fit. Higher values 
	// mean a worse fit. For each direction residual vectors' 
	// corresponding component is squared and accumulated.
	bool get_fitness_scores(double &y, double &z, double &yaw);
	// This function returns the sensor data map point correspondences.
	// Sensor points are at their transformed locations determined after
	// ICP. This function is intended for visualization purposes.
	bool get_correspondences(vector<pcl::PointXYZ> &sensor_pts, vector<pcl::PointXYZ> &map_pts);
};


#endif
