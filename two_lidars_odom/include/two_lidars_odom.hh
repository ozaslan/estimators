#ifndef __TWO_LIDARS_ODOM_HH__
#define __TWO_LIDARS_ODOM_HH__

#include <cmath>
#include <vector>
#include <limits>
#include <numeric>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <laser_proc.hh>
#include <utils.hh>

/*
	This class gets inputs from the two lidars scanners with a configuration
	as in the Inspection KHex platform and IMU data to estimate the incremental
	pose of the robot.
*/

class TwoLidarsOdom{
private:
	Eigen::Matrix4d _pose;
	int _max_iter;
	double _xyz_tol;
	double _rot_tol;
public:
	TwoLidarsOdom(int max_iter = 23, double xyz_tol = 0.001, double yaw_tol = 0.001);
	// This function provides an interface to provide laser scan
	// data to the odometry estimator. The 0th indices are assumed
	// to be older set of lidar data. The output is the two-frames
	// odometry. The global pose of the robot can be obtained by
	// concatenation of two-frames odometry estimates over time
	// by the wrapper. This class holds its own internal pose estimate
	// which can be retrieved through 'get_pose(...)' function.
	// 'init_pose' is the initial pose estimate. This function also
	// assumes that 3D points are generated but not transformed.
	// If 'switch_order == true' then the second element in the 
	// input arrays are taken as the first (older) lidar data.
	Eigen::Matrix4d estimate_odometry(const LaserProc bottom_laser_proc[2], 
									  const LaserProc    top_laser_proc[2], 
									  const Eigen::Matrix4d &init_pose, 
									  bool reset_pose = false,
									  bool switch_order = false);
	// This function returns the interval accumulated two-frames
	// odometry estimates, in other words the global pose of the 
	// robot.
	bool get_pose(Eigen::Matrix4d &pose){
		pose = _pose;
		return true;
	}
};


#endif
