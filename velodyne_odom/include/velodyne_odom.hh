#ifndef _VELODYNE_ODOM_HH_
#define _VELODYNE_ODOM_HH_

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>


#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

/*
	This class uses PCL functions to align point cloud from Velodyne 
	range sensor. This also accumulates the previous point cloud data
	to generate a map of the environment. The IMU data when available
	provides a prior for the orientation of the sensor to help the 
	registration process.

	-- Add a parameter structures and getter/setter
	-- Loop closing and optimization
	-- Overload '=' operator for VelodyneOdomParams struct so that it checks the values before assignment
*/

class VelodyneOdom{
public:
	struct VelodyneOdomParams{
		// Downsampling parameters for faster point registration
		double approximate_voxel_leaf_size[3];
		// Normal distribution transform (NDT) parameters for odometry.
		double ndt_eps;
		double ndt_step_size;
		double ndt_res;
		int    ndt_max_iter;
		// NDT params for batch optimization
		double batch_ndt_eps;
		double batch_ndt_step_size;
		double batch_ndt_res;
		int    batch_ndt_max_iter;

		// The two paramters below are used to find the weighted pose differences
		// in the corresponding directions. 
		// double trans_offset_weight;  // 
		// double rot_offset_weight[3]; // in [r, p, y] order
		// The below threshold values are compared to pose differences when
		// checking requirement for pushing new keyframes.
		double init_keyframe_trans_thres;
		double init_keyframe_rot_thres;
	
		VelodyneOdomParams() :	approximate_voxel_leaf_size({0.5, 0.5, 0.5}),
								ndt_eps(0.01), 
								ndt_step_size(0.1), 
								ndt_res(1.0), 
								ndt_max_iter(7),
								batch_ndt_eps(0.01), 
								batch_ndt_step_size(0.1), 
								batch_ndt_res( 1.0), 
								batch_ndt_max_iter ( 50),
								init_keyframe_trans_thres(1.0), // cm
								init_keyframe_rot_thres(DEG2RAD(10)) // radians
								{}
		int print();
	};
private:
	// Map of the environment as built by registering PC's
	pcl::PointCloud<pcl::PointXYZ>::Ptr _map;
	// Local subset of _map used for point cloud registration. This is used to speed up the process.
	pcl::PointCloud<pcl::PointXYZ>::Ptr _local_map;

	// Keyframes and their poses in the '_map' frame
	int _curr_keyframe_ind; // The index of the last used keyframe index
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _keyframes;
	std::vector<Eigen::Matrix4d> _keyframe_poses;

	// Current and filtered (downsampled) PC
	pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, _filtered_pc, _aligned_pc;

	// Pose of the last registered pc in the '_map' frame
	Eigen::Matrix4d _pc_pose;

	// Downsampling mechanism for fasted point registration
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> _approximate_voxel_filter;
	pcl::VoxelGrid<pcl::PointXYZ> _voxel_filter;
	pcl::PassThrough<pcl::PointXYZ> _pass_filter;

	// Registration mechanism	
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> _ndt;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> _batch_ndt;

	VelodyneOdomParams _params;
	
	gtsam::NonlinearFactorGraph _gtsam_graph_with_prior;
	gtsam::GaussNewtonParams _gtsam_params;
	gtsam::noiseModel::Diagonal::shared_ptr _gtsam_prior_model;

	std::vector<std::pair<int, int> > _pose_graph_edges;

	// This flag is set to 'true' when the batch optimization is still working.
	bool _batch_optimizing;

	// Parameters and vector initializations.
	void _initialize();
	// This function calculates the weighted distance between two poses. This is
	// used in finding the closest keyframe to the current point cloud
	double _pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2);
	double _pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double trans_thres, double rot_thres);
	// This function checks whether 'curr_pose' is far from the 'keyframe_ind'th
	// keyframe. This returns 'true' if the relative rotation as in roll-pitch-yaw
	// representation or Euclidean displacement is more than '_init_keyframe_trans_thres'
	// or '_init_keyframe_rot_thres'; otherwise returns false. Due to the anisotropic 
	// FOV of the 3D lidar, roll and pitch angles weigh more compared to the yaw angle.
	// The threshold is compared to the greatest of these angles. If 'key_frame_ind' <
	// '_keyframes.size()' does not hold, the function fails the assertion and exits
	// the program.
	bool _push_new_keyframe_required(const Eigen::Matrix4d &curr_pose, int key_frame_ind);
	// This function returns the index of the keyframe which has the least weighted sum
	// differences of the Euclidean and rotational offset from 'curr_pose'. The weights
	// are defined with the parameters '_trans_offset_weight', '_ros_offset_weight[3]' 
    // with [r, p, y] order. This function uses '_push_new_keyframe_required(...)' for
	// atomic checks. 
	int _find_closest_keyframe(const Eigen::Matrix4d &curr_pose);
	// This function optimized the alignment of the last 'num_keyframes' keyframes
	// in a separate thread.
	int _batch_optimize(int num_keyframes);
	// This function optimizes for the keyframes between '[start, end]_keyframe_ind'
	// indices inclusive. 
	int _batch_optimize(int start_keyframe_ind, int end_keyframes_ind);
	// This function batch optimizes only for the keyframes with indices given in the
	// 'keyframe_indices' vector.
	int _batch_optimize(const std::vector<int> &keyframe_indices);
	// This function stop the thread optimizing the keyframe poses. This can
	// be used if another optimization should be started and the current one
	// should be discarded. If 'undo_changes == true' then the '_keyframe_poses'
	// are kept as their values before the optimization. Otherwise updates
	// upto the time of cancellation is reflected on the poses.
	int _cancel_batch_optimization(bool undo_changes = true);
public:	
	VelodyneOdom();
	VelodyneOdom(const VelodyneOdomParams &param);
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
	const pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map(){ return _local_map;}
	const pcl::PointCloud<pcl::PointXYZ>::Ptr get_aligned_pc(){ return _aligned_pc;}
	const pcl::PointCloud<pcl::PointXYZ>::Ptr get_keyframe_pc(){ return _keyframes[_curr_keyframe_ind];}
	const std::vector<Eigen::Matrix4d> & get_keyframe_poses(){ return _keyframe_poses; }
	const std::vector<std::pair<int, int> > get_pose_graph_edges(){ return _pose_graph_edges;}
};

#endif
