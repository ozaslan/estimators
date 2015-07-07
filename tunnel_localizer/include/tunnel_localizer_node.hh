#include <list>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <utils.hh>
#include <lidar_calib_params.hh>
#include <camera_calib_params.hh>

#include "tunnel_localizer.hh"

/*
   -- Process inputs
   -- camera, laser, asus location yaml files (extrinsic calibration)
   -- The above should also include intrinsics calibration as well
   -- Setup messaging interface
   -- laser, imu, rgbd, cameras, 
   -- odom, visualization stuff
   -- Loop
   -- check sensor times before integration
 */

vector<pcl::PointXYZ> tails, tips;

// --------------------- PARAMETERS ----------------------------------------------------------- //
// * Estimation related parameters
Eigen::Matrix4d init_pose,
				estm_pose;
Eigen::Matrix6d estm_cov;
// * Configuration paramters
double	refresh_rate;
bool	debug_mode;
// * Laser processing parameters
int		median_filter_window	 = 5;
int		rate_linearity_window	 = 15;
double	downsample_thres		 = 0.03;
double	remove_occlusion_thres	 = 0.5;
int		remove_occlusion_window  = 3;
int		cluster_min_skips		 = 3;
double	cluster_range_jump		 = 0.50;
int		cluster_min_cluster_size = 3;
double	extract_lines_min_len	 = 0.15;
int		extract_lines_min_pts	 = 3;
double	extract_lines_epsilon	 = 0.08;
double	linearity_thres			 = 0.3;

// -------- COMMUNICATION INTERFACE DECLERATIONS ---------------------------------------------- //
// * Required publication channels
ros::Publisher odom_publ;
// * Required subscription channels
ros::Subscriber 
	imu_subs,
	top_lidar_subs,	bot_lidar_subs,
	top_cam_subs, bottom_cam_subs, 
	right_cam_subs, left_cam_subs,
	grid_map_sub, odom_subs;
// * Debugging channels
ros::Publisher  
	laser_pc_publ,
	res_rays_publ,
	map_pc_publ,
	flow_rays_publ;
// * Message declerations (### I might want to declare these locally)
sensor_msgs::LaserScan bot_lidar_msg, top_lidar_msg;
sensor_msgs::Image	top_cam_msg, bot_cam_msg, 
	right_cam_msg, left_cam_msg;
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg_in, odom_msg_out;
visualization_msgs::Marker res_rays_msg, flow_rays_msg;

// -------------- CLASS DECLERATIONS ---------------------------------------------------------- //
// * LaserProc etc.
LaserProc top_lidar_proc, bottom_lidar_proc;
bool is_top_lidar_valid, is_bottom_lidar_valid;
// * Calibration data is given as an argument together with
//   the corresponding sensor data.
CameraCalibParams top_cam_calib_params;
CameraCalibParams right_cam_calib_params;
CameraCalibParams bottom_cam_calib_params;
CameraCalibParams left_cam_calib_params;
LidarCalibParams  top_lidar_calib_params;
LidarCalibParams  bottom_lidar_calib_params;
// * Map of the environement
pcl::PointCloud<pcl::PointXYZ>::Ptr grid_map;
// * TunnelLocalizer class wraps around the range based localizer and
//   image based localizer. Through pushing sensor data, corresponding
//   localizer is populated with sensor data. This continues until a
//   call to pose estimate routine which then flushes saved data.
TunnelLocalizer tunnel_localizer;

// ----------------- CALLBACK DEFINITIONS ----------------------------------------------------- //
// An estimator is basically a process block which gets sensor
// data as inputs and outputs an estimate. Also it can take
// an initial value to pre-condition the estimator. The below functions
// constitute the communication interface for sensor inputs.
// We have 2 x lidars, 4 x cameras and an IMU.
void imu_callback(const sensor_msgs::Imu &msg);
void top_lidar_callback	(const sensor_msgs::LaserScan &msg);
void bot_lidar_callback	(const sensor_msgs::LaserScan &msg);
void top_cam_callback	(const sensor_msgs::Image &msg);
void bottom_cam_callback(const sensor_msgs::Image &msg);
void right_cam_callback	(const sensor_msgs::Image &msg);
void left_cam_callback	(const sensor_msgs::Image &msg);
void odom_callback		(const nav_msgs::Odometry &msg);

// ----------------- FUNCTION DEFINITIONS ----------------------------------------------------- //
void process_inputs(const ros::NodeHandle &n);
void setup_messaging_interface(ros::NodeHandle &n);
void loop();
void iterate_estimator();

void publish_odom();
void publish_res_rays();
void publish_flow_rays();
void publish_map();
void publish_laser();
