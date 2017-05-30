#ifndef _VELODYNE_ODOM_HH_
#define _VELODYNE_ODOM_HH_

#include <vector>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
// ### #include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

/*
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
*/

#include <utils.hh>

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
		double voxel_leaf_size[3];
		// Normal distribution transform (NDT) parameters for odometry.
		double ndt_eps;
		double ndt_step_size;
		double ndt_res;
		int    ndt_max_iter;
		double ndt_fitness_score_thres;
		// NDT params for batch optimization
		double batch_ndt_eps;
		double batch_ndt_step_size;
		double batch_ndt_res;
		int    batch_ndt_max_iter;

		// Maximum number of points for '_local_map'.
		// This is set for speed concerns.
		int local_map_max_points;

		// Dimensions of the bounding box '_local_map' centered at the current pose. 
		// Points outside this box are filtered.
		double local_map_dims[3];

		// The two paramters below are used to find the weighted pose differences
		// in the corresponding directions. 
		// double trans_offset_weight;  // 
		// double rot_offset_weight[3]; // in [r, p, y] order
		// The below threshold values are compared to pose differences when
		// checking requirement for pushing new keyframes.
		double init_keyframe_trans_thres;
		double init_keyframe_rot_thres;
		// Either of "NDT", "ICP", "NICP"
		string method;

		bool icp_use_reciprocal_corr;
		double icp_max_corr_dist;
		int icp_max_iter;
		double icp_trans_eps;
		double icp_euc_fitness_eps;

		bool   nicp_use_reciprocal_corr;
		double nicp_max_corr_dist;
		int    nicp_max_iter;
		double nicp_trans_eps;
		double nicp_euc_fitness_eps;

		double gicp_rot_eps;
		int    gicp_corr_randomness; // The number of neighbors used for covariances computation.
		int    gicp_max_iter;
		double gicp_max_corr_dist;

		VelodyneOdomParams() :	voxel_leaf_size({0.15, 0.15, 0.15}),
								ndt_eps(0.01), 
								ndt_step_size(0.1), 
								ndt_res(1.0), 
								ndt_max_iter(7),
								ndt_fitness_score_thres(1200.0),
								batch_ndt_eps(0.01), 
								batch_ndt_step_size(0.1), 
								batch_ndt_res( 1.0), 
								batch_ndt_max_iter ( 50),
								init_keyframe_trans_thres(1.00), // m
								init_keyframe_rot_thres(DEG2RAD(10)), // radians
								local_map_dims({40, 40, 40}), // m
								local_map_max_points(7000),
								method("NDT"),
								icp_use_reciprocal_corr(true),
								icp_max_corr_dist(1.5),
								icp_max_iter(50),
								icp_trans_eps(1e-8),
								icp_euc_fitness_eps(1),
								nicp_use_reciprocal_corr(true),
								nicp_max_corr_dist(1.5),
								nicp_max_iter(50),
								nicp_trans_eps(1e-8),
								nicp_euc_fitness_eps(1),
								gicp_rot_eps(0.001),
								gicp_corr_randomness(3),
								gicp_max_iter(50),
								gicp_max_corr_dist(1)
								{}
		int print();
	};
private:
  double _kf_time;
  double _temporal_kf_interval;
  bool   _temporal_kf_mode;

	std::string _methods_list;
	// Map of the environment as built by all keyframe pointclouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr _map;
	// Local subset of _map used for point cloud registration. This is used to speed up the process.
	// A subset of keyframe pointclouds in the vicinity of the current pose estiamte is used to
	// build this.
	pcl::PointCloud<pcl::PointXYZ>::Ptr _local_map;

	// Keyframes and their poses in the '_map' frame
	int _curr_keyframe_ind; // The index of the last used keyframe
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _keyframes;
	std::vector<Eigen::Matrix4d> _keyframe_poses;

	// Most recently pushed ('_pc'), filtered (downsampled, '_filtered_pc') 
	// and aligned ('_aligned') pointclouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, _filtered_pc, _aligned_pc;

	// Pose of the last registered pc in the '_map' frame
	Eigen::Matrix4d _pc_pose;

	// Downsampling mechanism for fasted point registration
	pcl::VoxelGrid<pcl::PointXYZ> _voxel_filter;
	// Pass filter is used to exclude points far from the current pose estimate
	pcl::PassThrough<pcl::PointXYZ> _pass_filter;
	Eigen::Vector3d _local_map_bbox;

	// Registration mechanism	
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> _ndt;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> _batch_ndt;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> _nicp;
	// ### pcl::GeneralizedIterativeClosestPoint6d<pcl::PointXYZ, pcl::PointXYZ> _gicp;
  
	VelodyneOdomParams _params;

  /*
	// GTSam mechanism (To be implemented later!)	
	gtsam::NonlinearFactorGraph _gtsam_graph_with_prior;
	gtsam::GaussNewtonParams _gtsam_params;
	gtsam::noiseModel::Diagonal::shared_ptr _gtsam_prior_model;
  */

	// This keeps track of which keyframes are fine-aligned with respect to
	// each other. Pose-graph as in _gtsam_graph_with_prior has the same
	// connectivity scheme with this.
	std::vector<std::pair<int, int> > _keyframe_connectivity;
	
	std::vector<std::pair<int, double> > _pose_keyframes_distances;
	std::vector<std::pair<int, double> > _prev_pose_keyframes_distances;

	// An unrealistic replacement for covariance :)
	Eigen::Matrix6d _cov;

	// This flag is set to 'true' when the batch optimization is still on.
	bool _batch_optimizing;

	// Parameters and vector initializations.
	void _initialize();
	// This function calculates the weighted distance between two poses. This is
	// used in finding the closest keyframe to the current point cloud.
	double _pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2);
	double _pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double trans_thres, double rot_thres);
	// This function checks whether 'curr_pose' is far from the 'keyframe_ind'th
	// keyframe. This returns 'true' if the weigthed average of the relative rotation 
	// as in roll-pitch-yaw representation and the Euclidean displacement is more than 
	// 1.0;  otherwise returns false. The weights are defined by 'init_keyframe_trans_thres'
	// and 'init_keyframe_rot_thres' parameters as in VelodyneOdomParams. Due to the anisotropic 
	// FOV of the 3D lidar, only roll and pitch angles are considered and yaw is discarded.
	// If 'key_frame_ind' < '_keyframes.size()' does not hold, the function fails the assertion 
	// and exits the program.
	bool _push_new_keyframe_required(const Eigen::Matrix4d &curr_pose, int key_frame_ind);
	// This function returns the index of the keyframe which has the least weighted sum
	// differences of the Euclidean and rotational offset from 'curr_pose'. The weights
	// are defined though the parameters 'init_keyframe_trans_thres' and 'init_keyframe_rot_thres' 
	int _find_closest_keyframe(const Eigen::Matrix4d &curr_pose);
	// This function optimizes the alignment of the last 'num_keyframes' keyframes
	// in a separate thread.
	int _batch_optimize(int num_keyframes);
	// This function optimizes for the keyframes between '[start, end]_keyframe_ind'
	// indices inclusive. 
	int _batch_optimize(int start_keyframe_ind, int end_keyframes_ind);
	// This function batch optimizes only for the keyframes with indices given in the
	// 'keyframe_indices' vector. (to be implemented)
	int _batch_optimize(const std::vector<int> &keyframe_indices);
	// This function stops the thread optimizing the keyframe poses. This can
	// be used if another optimization should be started and/or the current one
	// should be discarded. If 'undo_changes == true' then the '_keyframe_poses'
	// are kept as their values before the optimization. Otherwise updates
	// upto the time of cancellation is reflected on the poses.
	int _cancel_batch_optimization(bool undo_changes = true);
	// This function filters out the points in 'pc' outside the bounding box
	// centered at 'pos'. The results are overwritten to 'pc'. This function
	// returns the number of points in the filters pointcloud.
	int _trim_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const Eigen::Vector3d &pos, const Eigen::Vector3d &bbox);

	int _build_local_map(const Eigen::Matrix4d &pose);
	int _find_pose_keyframes_distances(const Eigen::Matrix4d &pose);
public:	
	VelodyneOdom();
	VelodyneOdom(const VelodyneOdomParams &param);
  inline bool set_temporal_kf_mode(bool kf_mode){ 
    _temporal_kf_mode = kf_mode; 
    return _temporal_kf_mode;
  }
  inline bool get_temporal_kf_mode(){return _temporal_kf_mode;}
  inline double set_temporal_kf_interval(double intervar){ 
    _temporal_kf_interval = intervar; 
    return _temporal_kf_interval;
  }
  inline double get_temporal_kf_interval(){return _temporal_kf_interval;}
	// This function gets a point cloud as input and stores it locally to
	// later register with '_local_map'. This happens when the programmer
	// explicitely calls one of the 'align(...)' functions. If the point cloud
	// is proper, this function returns '0', otherwise '-1'.
	int push_pc(const sensor_msgs::PointCloud2 &pc);
	int push_pc(const sensor_msgs::PointCloud2 &pc1, const sensor_msgs::PointCloud2 &pc2);
	int push_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
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
	// This function returns a constant pointer to the '_local_map' private
	// variable for visualization and other purposes.
	const pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map(){ return _local_map;}
	// This function returns a constant pointer to the '_aligned_pc' private
	// variable for visualization and other purposes. '_aligned_pc' is the
	// most recent pointcloud localized.
	const pcl::PointCloud<pcl::PointXYZ>::Ptr get_aligned_pc(){ return _aligned_pc;}
	// This function returns a constant pointer to the most recently used 
	// keyframe pointcloud. This can be used for visualization and other purposes. 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr get_keyframe_pc(){ return _keyframes[_curr_keyframe_ind];}
	// This function returns a constant reference to '_keyframe_poses' private
	// variable for visualization and other purposes.
	const std::vector<Eigen::Matrix4d> & get_keyframe_poses(){ return _keyframe_poses; }
	// This function returns a constant reference to the '_keyframe_connectivity' 
	// private variable for visualization and other purposes.
	const std::vector<std::pair<int, int> > get_keyframe_connectivity(){ return _keyframe_connectivity;}
	// This function returns the (unrealistic) covariance estimate for the
	// last pose estimation. '_cov' is basically a diagonal matrix elements of 
	// which are constant multiples of the fitness score as returned by the
	// pointcloud registration algorithm.
	Eigen::Matrix6d get_covariance(){ return _cov; }
};

#endif
