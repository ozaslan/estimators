#ifndef _VELODYNE_ODOM_HH_
#define _VELODYNE_ODOM_HH_

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>

/*
	This class uses PCL functions to align point cloud from Velodyne 
	range sensor. This also accumulates the previous point cloud data
	to generate a map of the environment. The IMU data when available
	provides a prior for the orientation of the sensor to help the 
	registration process.
*/

class VelodyneOdom{
private:
	// Map of the environment as built by registering PC's
	pcl::PointCloud<pcl::PointXYZ>::Ptr _map;
	// Keyframe and its pose in the '_map'
	pcl::PointCloud<pcl::PointXYZ>::Ptr _keyframe_pc;
	Eigen::Matrix4d _keyframe_pose;
	// Current and filtered (downsampled) PC
	pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, _filtered_pc, _aligned_pc;

	// Pose of the last registered pc in the '_map' frame
	Eigen::Matrix4d _pc_pose;
	double _approximate_voxel_leaf_size;
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> _approximate_voxel_filter;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> _ndt;

	void _initialize();
public:	
	VelodyneOdom();
	// This function gets a point cloud as input and stores it locally to
	// later register with the '_keyframe'. This happens when the programmer
	// explicitely calls one of the 'align(...)' functions. If the point cloud
	// is proper, this function returns '0', otherwise '-1'.
	int push_pc(const sensor_msgs::PointCloud2 &pc);
	// These functions align the most recently pushed pointcloud with the
	// keyframe. The programmer can give an initial estimate to help the
	// PCL alignment functions. If the alignment succeeds, this function
	// returns '0', otherwise '-1'
	int align();
	int align(const Eigen::Vector3d &init_pos);
	int align(const Eigen::Matrix3d &init_dcm);
	int align(const Eigen::Matrix4d &init_pose);
	// This function returns the last found pose estimate.
	Eigen::Matrix4d get_pose(){ return _pc_pose; }
	// This function returns a constant pointer to the '_map' private
	// variable for visualization and other purposes.
	const pcl::PointCloud<pcl::PointXYZ>::Ptr get_map(){ return _map;}
	const pcl::PointCloud<pcl::PointXYZ>::Ptr get_aligned_pc(){ return _aligned_pc;}
};

#endif
