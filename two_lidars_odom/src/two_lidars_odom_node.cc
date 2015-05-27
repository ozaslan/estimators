#include <list>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <utils.hh>
#include <calib_params.hh>
#include <laser_proc.hh>

#include "two_lidars_odom.hh"

ros::Publisher  odom_publ, laser_pc_publ;
ros::Subscriber imu_subs, top_lidar_subs, bottom_lidar_subs;

// Calibration data is given as an argument together with
// the corresponding sensor data.
LidarCalibParams  top_lidar_calib_params;
LidarCalibParams  bottom_lidar_calib_params;

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;
sensor_msgs::LaserScan top_lidar_msg, bottom_lidar_msg;
sensor_msgs::PointCloud pc_msg;

TwoLidarsOdom two_lidars_odom;
LaserProc top_lidar_proc[2], bottom_lidar_proc[2];
bool top_lidar_uptodate,
	 bottom_lidar_uptodate;
int  step;

double	refresh_rate;
bool	debug_mode;
string	top_lidar_calib_file;
string	bottom_lidar_calib_file;
// Parameters for LaserProc
bool	median_filter;
int		median_filter_window;
bool	downsample;
double	downsample_thres;
bool	remove_occlusion;
double	remove_occlusion_thres;
int		remove_occlusion_window;
bool	remove_slant_edges;
double	remove_slant_edges_thres;
int		remove_slant_edges_window;
bool	cluster;
int		cluster_min_skips;
double	cluster_range_jump;
int		cluster_min_cluster_size;
bool	rate_linearity;
int		rate_linearity_window;
bool	extract_lines;
double	extract_lines_min_len;
int		extract_lines_min_pts;
double	extract_lines_epsilon;
bool	project;
// Parameters for odometry
int		max_iter;
double	xyz_tol, rot_tol;

Eigen::Matrix4d trans, prev_trans;

// An estimator is basically a process block which gets sensor
// data as inputs and outputs an estimate. Also it can take
// an initial value to pre-condition the estimator. The below functions
// constitute the communication interface for sensor inputs.
// We have 2 x lidars and an IMU.
void imu_callback		  (const sensor_msgs::Imu &msg);
void top_lidar_callback	  (const sensor_msgs::LaserScan &msg);
void bottom_lidar_callback(const sensor_msgs::LaserScan &msg);

void publish_lidar_data();
void publish_odom();

void process_inputs(const ros::NodeHandle &n);
void setup_messaging_interface(ros::NodeHandle &n);
void loop(const ros::NodeHandle &n);

void iterate_odom();

int main(int argc, char* argv[]){

	ros::init(argc, argv, "two_lidars_odom_node");
	ros::NodeHandle n("~");

	trans = Eigen::Matrix4d::Identity();
	prev_trans = trans;

	top_lidar_uptodate = false;
	bottom_lidar_uptodate = false;
	step = 0;

	process_inputs(n);
	setup_messaging_interface(n);
	loop(n);	

	return 1;
}

void process_inputs(const ros::NodeHandle &n)
{
	n.param("refresh_rate", refresh_rate, 100.0);
	n.param("debug_mode", debug_mode, false);

	n.param("top_lidar_calib_file"	 ,	  top_lidar_calib_file, string("ERROR"));
	n.param("bottom_lidar_calib_file", bottom_lidar_calib_file, string("ERROR"));

	n.param("median_filter", median_filter, false);
	n.param("median_filter_window", median_filter_window, 3);
	n.param("downsample", downsample, false);
	n.param("downsample_thres", downsample_thres, 0.05);
	n.param("remove_occlusion", remove_occlusion, false);
	n.param("remove_occlusion_thres", remove_occlusion_thres, 0.50);
	n.param("remove_occlusion_window", remove_occlusion_window, 3);
	n.param("remove_slant_edges", remove_slant_edges, false);
	n.param("remove_slant_edges_thres", remove_slant_edges_thres, 1.4835);
	n.param("remove_slant_edges_window", remove_slant_edges_window, 3);
	n.param("cluster", cluster, false);
	n.param("cluster_min_skips", cluster_min_skips, 3);
	n.param("cluster_range_jump", cluster_range_jump, 0.50);
	n.param("cluster_min_cluster_size", cluster_min_cluster_size, 3);
	n.param("rate_linearity", rate_linearity, false);
	n.param("rate_linearity_window", rate_linearity_window, 3);
	n.param("extract_lines", extract_lines, false);
	n.param("extract_lines_min_len", extract_lines_min_len, 0.10);
	n.param("extract_lines_min_pts", extract_lines_min_pts, 3);
	n.param("extract_lines_epsilon", extract_lines_epsilon, 0.08);
	n.param("project", project, false);
	n.param("max_iter", max_iter, 33);
	n.param("xyz_tol", xyz_tol, 0.01);
	n.param("rot_tol", rot_tol, 1.0 / 180 * PI);

	ROS_INFO(" ---------- TWO LIDARS ODOM ------------");
	ROS_INFO("[refresh_rate] ------------- : [%.3f]", refresh_rate);
	ROS_INFO("[debug_mode] --------------- : [%s]"  , debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[top_lidar_calib_file] ------: [%s]"  , top_lidar_calib_file.c_str());
	ROS_INFO("[bottom_lidar_calib_file] -- : [%s]"  , bottom_lidar_calib_file.c_str());
	ROS_INFO("[median_filter] ------------ : [%s]"  , median_filter ? "TRUE" : "FALSE");
	ROS_INFO("[median_filter_window] ----- : [%d]"  , median_filter_window);
	ROS_INFO("[downsample] --------------- : [%s]"  , downsample ? "TRUE" : "FALSE");
	ROS_INFO("[downsample_thres] --------- : [%.3f]", downsample_thres);
	ROS_INFO("[remove_occlusion] --------- : [%s]"  , remove_occlusion ? "TRUE" : "FALSE");
	ROS_INFO("[remove_occlusion_thres] --- : [%.3f]", remove_occlusion_thres);
	ROS_INFO("[remove_occlusion_window] -- : [%d]"  , remove_slant_edges_window);
	ROS_INFO("[remove_slant_edges] ------- : [%s]"  , remove_slant_edges ? "TRUE" : "FALSE");
	ROS_INFO("[remove_slant_edges_thres -- : [%.3f]", remove_slant_edges_thres);
	ROS_INFO("[remove_slant_edges_window]  : [%d]"  , remove_slant_edges_window);
	ROS_INFO("[cluster] ------------------ : [%s]"  , cluster ? "TRUE" : "FALSE");
	ROS_INFO("[cluster_min_skips] -------- : [%d]"  , cluster_min_skips);
	ROS_INFO("[cluster_range_jump] ------- : [%.3f]", cluster_range_jump);
	ROS_INFO("[cluster_min_cluster_size] - : [%d]"  , cluster_min_cluster_size);
	ROS_INFO("[rate_linearity] ----------- : [%s]"  , rate_linearity ? "TRUE" : "FALSE");
	ROS_INFO("[rate_linearity_window] ---- : [%d]"  , rate_linearity_window);
	ROS_INFO("[extract_lines] ------------ : [%s]"  , extract_lines ? "TRUE" : "FALSE");
	ROS_INFO("[extract_lines_min_len] ---- : [%.3f]", extract_lines_min_len);
	ROS_INFO("[extract_lines_min_pts] ---- : [%d]"  , extract_lines_min_pts);
	ROS_INFO("[extract_lines_epsilon] ---- : [%.3f]", extract_lines_epsilon);
	ROS_INFO("[project] ------------------ : [%s]"  , project ? "TRUE" : "FALSE");
	ROS_INFO("[max_iter] ----------------- : [%d]"  , max_iter);
	ROS_INFO("[xyz_tol, rot_tol] --------- : [%.4f, %.4f]", xyz_tol, rot_tol);
	ROS_INFO(" ---------------------------------------");

	top_lidar_calib_params.load(top_lidar_calib_file);
	ROS_INFO("Top Lidar Calib Params :");
	top_lidar_calib_params.print();

	bottom_lidar_calib_params.load(bottom_lidar_calib_file);
	ROS_INFO("Bottom Lidar Calib Params :");
	bottom_lidar_calib_params.print();

	//sm_debug_write(0);
}

void setup_messaging_interface(ros::NodeHandle &n)
{
	// A proper estimator node is supposed to take "only sensor inputs" and
	// "output an estimate for X". This node's inputs are IMU, 2 x Scan and 
	// 4 x Cams. Its output is the relative pose of the robot w.r.t. to the
	// given map (if any).
	odom_publ         = n.advertise<nav_msgs::Odometry>("odom", 10);
	imu_subs          = n.subscribe("imu", 10    , imu_callback, ros::TransportHints().tcpNoDelay()); 
	top_lidar_subs    = n.subscribe("scan/top"   , 10,    top_lidar_callback, ros::TransportHints().tcpNoDelay()); 
	bottom_lidar_subs = n.subscribe("scan/bottom", 10, bottom_lidar_callback, ros::TransportHints().tcpNoDelay()); 

	// Below are the 'debug' outputs. These (in/out)puts are not inevitable 
	// for the proper execution of the node. However for visualization and
	// debugging purposes, they provide convenience.
	laser_pc_publ = n.advertise<sensor_msgs::PointCloud>("debug/pc", 10);
}

void loop(const ros::NodeHandle &n)
{
	ros::Rate r(refresh_rate);

	while (n.ok())
	{
		r.sleep();
		ros::spinOnce();
	}
}

void imu_callback(const sensor_msgs::Imu &msg){
	if(debug_mode)
		ROS_INFO("TWO LIDARS ODOM : Got IMU Data");

	imu_msg = msg;

	imu_msg.orientation.x *= -1;
	imu_msg.orientation.y *= -1;
}

void bottom_lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("TWO LIDARS ODOM : Got Bottom Lidar Data");

	bottom_lidar_msg = msg;

	bottom_lidar_proc[step % 2].update_data(bottom_lidar_msg, bottom_lidar_calib_params);
	if(median_filter == true)
		bottom_lidar_proc[step % 2].median_filter(median_filter_window);
	if(rate_linearity == true)
		bottom_lidar_proc[step % 2].rate_linearity(rate_linearity_window);
	if(downsample == true)                                                                                                                                              
		bottom_lidar_proc[step % 2].downsample(downsample_thres);
	if(remove_occlusion == true)
		bottom_lidar_proc[step % 2].remove_occlusions(remove_occlusion_thres, remove_occlusion_window);
	if(remove_slant_edges == true)
		bottom_lidar_proc[step % 2].remove_slant_edges(remove_slant_edges_thres, remove_slant_edges_window);
	if(cluster == true)
		bottom_lidar_proc[step % 2].cluster(cluster_min_skips, cluster_range_jump, cluster_min_cluster_size);
	if(extract_lines == true)
		bottom_lidar_proc[step % 2].extract_lines(extract_lines_min_len, extract_lines_min_pts, extract_lines_epsilon);

	bottom_lidar_proc[step % 2].transform(Eigen::Matrix4d::Identity(), true);

	bottom_lidar_uptodate = true;

	iterate_odom();
}

void top_lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("TWO LIDARS ODOM : Got Top Lidar Data");

	top_lidar_msg = msg;

	top_lidar_proc[step % 2].update_data(top_lidar_msg, top_lidar_calib_params);
	if(median_filter == true)
		top_lidar_proc[step % 2].median_filter(median_filter_window);
	if(rate_linearity == true)
		top_lidar_proc[step % 2].rate_linearity(rate_linearity_window);
	if(downsample == true)                                                                                                                                              
		top_lidar_proc[step % 2].downsample(downsample_thres);
	if(remove_occlusion == true)
		top_lidar_proc[step % 2].remove_occlusions(remove_occlusion_thres, remove_occlusion_window);
	if(remove_slant_edges == true)
		top_lidar_proc[step % 2].remove_slant_edges(remove_slant_edges_thres, remove_slant_edges_window);
	if(cluster == true)
		top_lidar_proc[step % 2].cluster(cluster_min_skips, cluster_range_jump, cluster_min_cluster_size);
	if(extract_lines == true)
		top_lidar_proc[step % 2].extract_lines(extract_lines_min_len, extract_lines_min_pts, extract_lines_epsilon);

	top_lidar_proc[step % 2].transform(Eigen::Matrix4d::Identity(), true);

	top_lidar_uptodate = true;

	iterate_odom();
}

void iterate_odom(){
	if(bottom_lidar_uptodate == false || top_lidar_uptodate == false)
		return;

	bottom_lidar_uptodate	= false;
	top_lidar_uptodate		= false;

	if(step < 2){
	} else {
		trans.topLeftCorner<3, 3>() = utils::trans::imu2dcm(imu_msg);
		Eigen::Matrix4d dtrans = trans.inverse() * prev_trans;
		two_lidars_odom.estimate_odometry(bottom_lidar_proc, top_lidar_proc, dtrans, false, (step % 2) == 0);
		prev_trans = trans;
	}

	publish_odom();
	publish_lidar_data();
	
	step++;
	return;
}

void publish_odom(){
	Eigen::Matrix4d pose;
	Eigen::Vector3d rpy_odom, rpy_imu;
	two_lidars_odom.get_pose(pose);

	rpy_odom = utils::trans::dcm2rpy(Eigen::Matrix3d(pose.topLeftCorner<3, 3>()));
	rpy_imu  = utils::trans::imu2rpy(imu_msg);
	rpy_odom(0) = rpy_imu(0);
	rpy_odom(1) = rpy_imu(1);
	pose.topLeftCorner<3, 3>() = utils::trans::rpy2dcm(rpy_odom);

	odom_msg = utils::trans::se32odom(pose);
	odom_msg.header.seq++;
	odom_msg.header.frame_id = "world";
	odom_msg.header.stamp = ros::Time::now();
	odom_publ.publish(odom_msg);
}

void publish_lidar_data(){
	Eigen::Matrix4d pose;
	two_lidars_odom.get_pose(pose);

	Eigen::Vector3d rpy_odom, rpy_imu;
	rpy_odom = utils::trans::dcm2rpy(Eigen::Matrix3d(pose.topLeftCorner<3, 3>()));
	rpy_imu  = utils::trans::imu2rpy(imu_msg);
	rpy_odom(0) = rpy_imu(0);
	rpy_odom(1) = rpy_imu(1);
	//rpy_odom(2) = 0;
	pose.topLeftCorner<3, 3>() = utils::trans::rpy2dcm(rpy_odom);

	   top_lidar_proc[step % 2].transform(pose, false);
	bottom_lidar_proc[step % 2].transform(pose, false);

	const vector<Eigen::Vector3d>    top_3d_rays =    top_lidar_proc[step % 2].get_3d_points();
	const vector<Eigen::Vector3d> bottom_3d_rays = bottom_lidar_proc[step % 2].get_3d_points();
	const vector<double>    top_linearities =    top_lidar_proc[step % 2].get_linearity_rates();
	const vector<double> bottom_linearities = bottom_lidar_proc[step % 2].get_linearity_rates();
	const vector<int>    top_mask =    top_lidar_proc[step % 2].get_mask();
	const vector<int> bottom_mask = bottom_lidar_proc[step % 2].get_mask();

	pc_msg.header.seq++;
	pc_msg.header.frame_id = "world";
	pc_msg.header.stamp = ros::Time::now();

	pc_msg.points.clear();
	pc_msg.points.reserve(top_3d_rays.size() + bottom_3d_rays.size());
	for(int i = 0 ; i < (int)top_3d_rays.size() ; i++){
		if(top_linearities[i] <= 0.3 && top_mask[i] != 0){
			geometry_msgs::Point32 pt;
			pt.x = top_3d_rays[i](0);
			pt.y = top_3d_rays[i](1);
			pt.z = top_3d_rays[i](2);
			pc_msg.points.push_back(pt);
		}
	}

	int offset = top_3d_rays.size();
	for(int i = 0 ; i < (int)bottom_3d_rays.size() ; i++){
		if(bottom_linearities[i] <= 0.3 && bottom_mask[i] != 0){
			geometry_msgs::Point32 pt;
			pt.x = bottom_3d_rays[i](0);
			pt.y = bottom_3d_rays[i](1);
			pt.z = bottom_3d_rays[i](2);
			pc_msg.points.push_back(pt);
		}
	}

	laser_pc_publ.publish(pc_msg);
}


