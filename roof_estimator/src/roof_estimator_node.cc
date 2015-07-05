#include <list>
#include <string>
#include <iostream>

#include <ros/ros.h>
//#include <octomap_ros/conversions.h>
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

#include "roof_localizer.hh"
#include "roof_mapper.hh"

vector<pcl::PointXYZ> tails, tips;

double refresh_rate;
bool   debug_mode;

ros::Publisher  odom_publ, 
	laser_pc_publ,
	res_rays_publ,
	map_publ,
	flow_rays_publ,
	last_range_registration_publ;
ros::Subscriber imu_subs,
	top_lidar_subs,	bot_lidar_subs,
	top_cam_subs, bottom_cam_subs, 
	right_cam_subs, left_cam_subs,
	grid_map_sub, odom_subs;
// ### I might want to have a few more publisher for
// visualization.

// Calibration data is given as an argument together with
// the corresponding sensor data.
CameraCalibParams top_cam_calib_params;
CameraCalibParams right_cam_calib_params;
CameraCalibParams bottom_cam_calib_params;
CameraCalibParams left_cam_calib_params;
LidarCalibParams  top_lidar_calib_params;
LidarCalibParams  bottom_lidar_calib_params;

// RoofLocalizer class wraps around the range based localizer and
// image based localizer. Through pushing sensor data, corresponding
// localizer is populated with sensor data. This continues until a
// call to pose estimate routine which then flushes saved data.
RoofLocalizer roof_localizer;
RoofMapper    roof_mapper;

sensor_msgs::LaserScan bot_lidar_msg, top_lidar_msg;
sensor_msgs::Image	top_cam_msg, bot_cam_msg, 
	right_cam_msg, left_cam_msg;

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg_in, odom_msg_out;
visualization_msgs::Marker res_rays_msg, flow_rays_msg, laser_out_msg;
octomap_msgs::Octomap octomap_msg;

int median_filter_window = 5;
int rate_linearity_window = 15;
double downsample_thres = 0.03;
double remove_occlusion_thres = 0.5;
int remove_occlusion_window = 3;
int cluster_min_skips = 3;
double cluster_range_jump = 0.50;
int cluster_min_cluster_size = 3;
double extract_lines_min_len = 0.15;
int extract_lines_min_pts = 3;
double extract_lines_epsilon = 0.08;
double linearity_thres = 0.3;

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

// ------------------------------------------------------- //
void process_inputs(const ros::NodeHandle &n);
void setup_messaging_interface(ros::NodeHandle &n);
void loop(const ros::NodeHandle &n);
void iterate_estimator();

void publish_res_rays();
void publish_flow_rays();
void publish_odom();
void publish_map();
void publish_laser_rays();

Eigen::Matrix4d estm_pose;
Eigen::Matrix6d estm_cov;

LaserProc top_lidar_proc, bottom_lidar_proc;
bool is_top_lidar_valid, is_bottom_lidar_valid;

int main(int argc, char* argv[]){

	ros::init(argc, argv, "roof_estimator_node");
	ros::NodeHandle n("~");

	process_inputs(n);
	setup_messaging_interface(n);
	loop(n);	

	return 1;
}

void process_inputs(const ros::NodeHandle &n)
{
	n.param("refresh_rate", refresh_rate, 100.0);
	n.param("debug_mode", debug_mode, false);

	// Query for parameter file paths from the parameter server.
	string top_cam_calib_file;
	string right_cam_calib_file;
	string bottom_cam_calib_file;
	string left_cam_calib_file;
	string top_lidar_calib_file;
	string bottom_lidar_calib_file;

	n.param("top_lidar_calib_file"	, top_lidar_calib_file	 , string("ERROR"));
	n.param("bottom_lidar_calib_file"	, bottom_lidar_calib_file, string("ERROR"));
	n.param("top_cam_calib_file"		, top_cam_calib_file	 , string("ERROR"));
	n.param("right_cam_calib_file"	, right_cam_calib_file	 , string("ERROR"));
	n.param("bottom_cam_calib_file"	, bottom_cam_calib_file	 , string("ERROR"));
	n.param("left_cam_calib_file"		, left_cam_calib_file	 , string("ERROR"));

	// ### I have to get laser and camera intrinsic/extrinsic 
	// parameters from the parameter server too.
	// These include the laser dead regions, noise models,
	// camera calibration, optical flow and feature extraction
	// related parameters, laser subsampling, laser max range
	// parameters, method to use in covariance calculation,
	// whether to use 1/2 lasers, rgbd, which cameras to use
	// camera exposure and exposure calibration (maybe) 
	// whether to have the lights on/off (we need a circuitry
	// for that), particle filter related parameters, failure case
	// behaviors, 


	ROS_INFO(" ---------- ROOF ESTIMATOR ------------");
	ROS_INFO("[refresh_rate] ---------- : [%.3f]", refresh_rate);
	ROS_INFO("[debug_mode] ------------ : [%s]", debug_mode ? "TRUE" : "FALSE");
	//ROS_INFO("[map_path] -------------- : [%s]", map_path.c_str());
	ROS_INFO("[top_lidar_calib_file] ---: [%s]", top_lidar_calib_file.c_str());
	ROS_INFO("[bottom_lidar_calib_file] : [%s]", bottom_lidar_calib_file.c_str());
	ROS_INFO("[top_cam_calib_file] -----: [%s]", top_cam_calib_file.c_str());
	ROS_INFO("[right_cam_calib_file] ---: [%s]", right_cam_calib_file.c_str());
	ROS_INFO("[bottom_cam_calib_file] --: [%s]", bottom_cam_calib_file.c_str());
	ROS_INFO("[left_cam_calib_file] ----: [%s]", left_cam_calib_file.c_str());
	ROS_INFO(" ----------------------------------------");

	// Load calibration data for each sensor. 
	// ### (to be implemented) In case of load failure, 'is_valid'
	// flag is set to 'false'.

	top_lidar_calib_params.load(top_lidar_calib_file);
	//ROS_INFO("Top Lidar Calib Params :");
	top_lidar_calib_params.print();

	bottom_lidar_calib_params.load(bottom_lidar_calib_file);
	//ROS_INFO("Bottom Lidar Calib Params :");
	bottom_lidar_calib_params.print();

	top_cam_calib_params.load(top_cam_calib_file);
	//ROS_INFO("Top Cam Calib Params :");
	//top_cam_calib_params.print();

	right_cam_calib_params.load(right_cam_calib_file);
	//ROS_INFO("Right Cam Calib Params :");
	//right_cam_calib_params.print();

	bottom_cam_calib_params.load(bottom_cam_calib_file);
	//ROS_INFO("Bottom Cam Calib Params :");
	//bottom_cam_calib_params.print();

	left_cam_calib_params.load(left_cam_calib_file);
	//ROS_INFO("Left Cam Calib Params :");
	//left_cam_calib_params.print();

	is_top_lidar_valid		= false;
	is_bottom_lidar_valid	= false;

}

void setup_messaging_interface(ros::NodeHandle &n)
{
	// A proper estimator node is supposed to take "only sensor inputs" and
	// "output an estimate for X". This node's inputs are IMU, 2 x Scan and 
	// 4 x Cams. Its output is the relative pose of the robot w.r.t. to the
	// given map (if any).
	odom_publ        = n.advertise<nav_msgs::Odometry>("odom", 10);
	imu_subs         = n.subscribe("imu"	    , 10, imu_callback		, ros::TransportHints().tcpNoDelay()); 
	top_lidar_subs   = n.subscribe("scan/top"   , 10, top_lidar_callback	, ros::TransportHints().tcpNoDelay()); 
	bot_lidar_subs   = n.subscribe("scan/bottom", 10, bot_lidar_callback	, ros::TransportHints().tcpNoDelay()); 
	top_cam_subs	 = n.subscribe("cam/top"    , 10, top_cam_callback	, ros::TransportHints().tcpNoDelay()); 
	bottom_cam_subs  = n.subscribe("cam/bottom" , 10, bottom_cam_callback	, ros::TransportHints().tcpNoDelay()); 
	right_cam_subs   = n.subscribe("cam/right"  , 10, right_cam_callback	, ros::TransportHints().tcpNoDelay()); 
	left_cam_subs    = n.subscribe("cam/left"   , 10, left_cam_callback	, ros::TransportHints().tcpNoDelay()); 
	// ### Add RGBD subscriber

	// Below are the 'debug' outputs. These (in/out)puts are not inevitable 
	// for the proper execution of the node. However for visualization and
	// debugging purposes, they provide convenience.
	laser_pc_publ	= n.advertise<visualization_msgs::Marker>("debug/aligned_sensor_data", 10);
	res_rays_publ   = n.advertise<visualization_msgs::Marker>("debug/residual_rays", 10);
	flow_rays_publ  = n.advertise<visualization_msgs::Marker>("debug/flow_rays", 10);
	last_range_registration_publ = n.advertise<sensor_msgs::PointCloud>("debug/pc/last_range_registration", 10);
	// The current implementation loads the map from a '.pcd' file.
	// In the following versions, we may swich to listening to octomap messages.
	map_publ		= n.advertise<octomap_msgs::Octomap>("debug/pc/map", 10);
}

void loop(const ros::NodeHandle &n)
{
	estm_pose = Eigen::Matrix4d::Identity();
	ros::spin();
}	

void imu_callback(const sensor_msgs::Imu &msg){
	if(debug_mode)
		ROS_INFO("ROOF ESTIMATOR : Got IMU Data");

	imu_msg = msg;

	Eigen::Vector3d rpy_imu  = utils::trans::imu2rpy(imu_msg);
	Eigen::Vector3d rpy_estm = utils::trans::dcm2rpy(Eigen::Matrix3d(estm_pose.topLeftCorner<3, 3>()));
	rpy_estm(0) = rpy_imu(0);
	rpy_estm(1) = rpy_imu(1);
	estm_pose.topLeftCorner<3, 3>() = utils::trans::rpy2dcm(rpy_estm);
}

void odom_callback(const nav_msgs::Odometry &msg){
	if(debug_mode)
		ROS_INFO("ROOF ESTIMATOR : Got Odometry Data");

	odom_msg_in = msg;

	estm_pose = utils::trans::odom2se3(odom_msg_in);
}

void bot_lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("ROOF ESTIMATOR : Got Bottom Lidar Data");

	bot_lidar_msg = msg;

	bottom_lidar_proc.update_data(bot_lidar_msg, bottom_lidar_calib_params);

	bottom_lidar_proc.median_filter(median_filter_window);
	bottom_lidar_proc.cluster(cluster_min_skips, cluster_range_jump, cluster_min_cluster_size);
	bottom_lidar_proc.downsample(downsample_thres);
	bottom_lidar_proc.remove_occlusions(remove_occlusion_thres, remove_occlusion_window);
	bottom_lidar_proc.rate_linearity(rate_linearity_window);
	bottom_lidar_proc.linearity_filter(linearity_thres);
	//bottom_lidar_proc.extract_lines(extract_lines_min_len, extract_lines_min_pts, extract_lines_epsilon);
	bottom_lidar_proc.transform(Eigen::Matrix4d::Identity(), true);
	bottom_lidar_proc.estimate_fim();
	is_bottom_lidar_valid = true;

	iterate_estimator();

}
void top_lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("ROOF ESTIMATOR : Got Top Lidar Data");

	top_lidar_msg = msg;

	top_lidar_proc.update_data(top_lidar_msg, top_lidar_calib_params);

	top_lidar_proc.median_filter(median_filter_window);
	top_lidar_proc.cluster(cluster_min_skips, cluster_range_jump, cluster_min_cluster_size);
	top_lidar_proc.downsample(downsample_thres);
	top_lidar_proc.rate_linearity(rate_linearity_window);
	top_lidar_proc.linearity_filter(linearity_thres);
	top_lidar_proc.remove_occlusions(remove_occlusion_thres, remove_occlusion_window);
	//top_lidar_proc.extract_lines(extract_lines_min_len, extract_lines_min_pts, extract_lines_epsilon);
	top_lidar_proc.transform(Eigen::Matrix4d::Identity(), true);
	top_lidar_proc.estimate_fim();
	is_top_lidar_valid = true;

	iterate_estimator();

}

void top_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("ROOF ESTIMATOR : Got Top Camera Data");

	top_cam_msg = msg;

	iterate_estimator();
}

void bottom_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("ROOF ESTIMATOR : Got Bottom Camera Data");

	bot_cam_msg = msg;

	iterate_estimator();
}

void right_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("ROOF ESTIMATOR : Got Right Camera Data");

	right_cam_msg = msg;

	iterate_estimator();
}

void left_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("ROOF ESTIMATOR : Got Left Camera Data");

	left_cam_msg = msg;

	iterate_estimator();
}

void iterate_estimator(){
	if(is_top_lidar_valid == false || is_bottom_lidar_valid == false)
		return;

	is_top_lidar_valid = false;
	is_bottom_lidar_valid = false;

	roof_localizer.push_lidar_data(bottom_lidar_proc, true);
	roof_localizer.push_lidar_data(top_lidar_proc);

	roof_localizer.estimate_pose(estm_pose, roof_mapper.get_octomap());
	roof_localizer.get_pose(estm_pose);
	roof_localizer.get_covariance(estm_cov);

	static bool is_first_frame = true;

	if(is_first_frame){
		roof_mapper.register_lidar_data(estm_pose, top_lidar_proc);
		const octomap::Pointcloud cloud1 = roof_mapper.get_last_range_registration();
		roof_mapper.register_lidar_data(estm_pose, bottom_lidar_proc);	
		const octomap::Pointcloud cloud2 = roof_mapper.get_last_range_registration();
		is_first_frame = false;
		// ------------------- FOR DEBUGGING START ----------------------------- //

		static sensor_msgs::PointCloud temp_cloud;
		temp_cloud.header.stamp = top_lidar_msg.header.stamp;
		temp_cloud.header.seq++;
		temp_cloud.header.frame_id = "world";
		temp_cloud.points.clear();
		temp_cloud.points.resize(cloud1.size());
		for(int i = 0 ; i < (int)cloud1.size() ; i++){
			octomap::point3d pt = cloud1.getPoint(i);
			temp_cloud.points[i].x = pt.x();
			temp_cloud.points[i].y = pt.y();
			temp_cloud.points[i].z = pt.z();
		}

		int offset = temp_cloud.points.size();
		temp_cloud.points.resize(offset + cloud2.size());
		for(int i = 0 ; i < (int)cloud2.size() ; i++){
			octomap::point3d pt = cloud2.getPoint(i);
			temp_cloud.points[offset + i].x = pt.x();
			temp_cloud.points[offset + i].y = pt.y();
			temp_cloud.points[offset + i].z = pt.z();
		}

		last_range_registration_publ.publish(temp_cloud);
	}


	bottom_lidar_proc.transform(estm_pose, false);
	top_lidar_proc.transform(estm_pose, false);

	publish_laser_rays();
	//publish_res_rays();
	//publish_flow_rays();
	publish_odom();
	publish_map();

	/*
	   static pcl::PointCloud<pcl::PointXYZ>::Ptr registered_lidar_cloud;
	   roof_localizer.get_registered_lidar_cloud(registered_lidar_cloud);
	   registered_lidar_cloud->header.frame_id = "world";
	   registered_lidar_cloud->header.stamp = ros::Time::now().toSec();
	   registered_lidar_cloud->header.seq++;

	   laser_pc_publ.publish(*registered_lidar_cloud);
	 */

	// ------------------- FOR DEBUGGING END ------------------------------- //
}

void publish_laser_rays(){
	laser_out_msg.header.seq++;
	laser_out_msg.header.frame_id = "world";
	laser_out_msg.header.stamp = ros::Time::now();

	laser_out_msg.id = 0;
	laser_out_msg.type = visualization_msgs::Marker::POINTS;
	laser_out_msg.action = visualization_msgs::Marker::ADD;
	laser_out_msg.pose.position.x = 0;
	laser_out_msg.pose.position.y = 0;
	laser_out_msg.pose.position.z = 0;
	laser_out_msg.pose.orientation.x = 0.0;
	laser_out_msg.pose.orientation.y = 0.0;
	laser_out_msg.pose.orientation.z = 0.0;
	laser_out_msg.pose.orientation.w = 1.0;
	laser_out_msg.scale.x = 0.1;
	laser_out_msg.scale.y = 0.1;
	laser_out_msg.scale.z = 0.1;
	laser_out_msg.color.a = 1.0;
	laser_out_msg.color.r = 1.0;
	laser_out_msg.color.g = 0.0;
	laser_out_msg.color.b = 0.0;

	const vector<Eigen::Vector3d> &top_3d_pts    =    top_lidar_proc.get_3d_points();
	const vector<Eigen::Vector3d> &bottom_3d_pts = bottom_lidar_proc.get_3d_points();
	const vector<int> &top_mask    =    top_lidar_proc.get_mask();
	const vector<int> &bottom_mask = bottom_lidar_proc.get_mask();

	laser_out_msg.points.clear();
	laser_out_msg.points.reserve(top_3d_pts.size() + bottom_3d_pts.size());
	for(int i = 0 ; i < (int)top_3d_pts.size() ; i++){
		geometry_msgs::Point pt;
		if(top_mask[i] > 0){
			pt.x = top_3d_pts[i](0);
			pt.y = top_3d_pts[i](1);
			pt.z = top_3d_pts[i](2);
			laser_out_msg.points.push_back(pt);
		}
	}
	for(int i = 0 ; i < (int)bottom_3d_pts.size() ; i++){
		geometry_msgs::Point pt;
		if(bottom_mask[i] > 0){
			pt.x = bottom_3d_pts[i](0);
			pt.y = bottom_3d_pts[i](1);
			pt.z = bottom_3d_pts[i](2);
			laser_out_msg.points.push_back(pt);
		}
	}

	laser_pc_publ.publish(laser_out_msg);
}

// This function visualizes the residual vectors after ICP.
// Hence it gives an idea on how well the RangeBasedRoofLocalizer
// performs.
void publish_res_rays(){
	vector<pcl::PointXYZ> range, map;
	roof_localizer.get_range_map_correspondences(range, map);

	res_rays_msg.header.seq++;
	res_rays_msg.header.frame_id = "world";
	res_rays_msg.header.stamp = ros::Time::now();
	res_rays_msg.lifetime = ros::Duration(0);

	res_rays_msg.id = 0;
	res_rays_msg.type = visualization_msgs::Marker::LINE_LIST;
	res_rays_msg.action = visualization_msgs::Marker::ADD;
	res_rays_msg.pose.position.x = 0;
	res_rays_msg.pose.position.y = 0;
	res_rays_msg.pose.position.z = 0;
	res_rays_msg.pose.orientation.x = 0.0;
	res_rays_msg.pose.orientation.y = 0.0;
	res_rays_msg.pose.orientation.z = 0.0;
	res_rays_msg.pose.orientation.w = 1.0;
	res_rays_msg.scale.x = 0.01;
	res_rays_msg.scale.y = 0.01;
	res_rays_msg.scale.z = 0.01;
	res_rays_msg.color.a = 1.0;
	res_rays_msg.color.r = 1.0;
	res_rays_msg.color.g = 0.0;
	res_rays_msg.color.b = 0.0;

	int num_pts = map.size();
	res_rays_msg.points.resize(2*num_pts);
	for(int i = 0 ; i < num_pts ; i++){
		res_rays_msg.points[2*i].x = map[i].x;
		res_rays_msg.points[2*i].y = map[i].y;
		res_rays_msg.points[2*i].z = map[i].z;
		res_rays_msg.points[2*i+1].x = range[i].x;
		res_rays_msg.points[2*i+1].y = range[i].y;
		res_rays_msg.points[2*i+1].z = range[i].z;
		if(map[i].x == std::numeric_limits<float>::infinity())
			res_rays_msg.points[2*i] = res_rays_msg.points[2*i+1];
	}
	res_rays_publ.publish(res_rays_msg);
}


void publish_flow_rays(){
	vector<pcl::PointXYZ> tails, pits;
	roof_localizer.get_back_projected_features(tails, tips);

	flow_rays_msg.header.seq++;
	flow_rays_msg.header.frame_id = "world";
	flow_rays_msg.header.stamp = ros::Time::now();
	flow_rays_msg.lifetime = ros::Duration(0.01);

	flow_rays_msg.id = 33;
	flow_rays_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
	flow_rays_msg.action = visualization_msgs::Marker::ADD;
	flow_rays_msg.pose.position.x = odom_msg_out.pose.pose.position.x - 2;
	flow_rays_msg.pose.position.y = 0;
	flow_rays_msg.pose.position.z = 0;
	flow_rays_msg.pose.orientation.x = 0.0;
	flow_rays_msg.pose.orientation.y = 0.0;
	flow_rays_msg.pose.orientation.z = 0.0;
	flow_rays_msg.pose.orientation.w = 1.0;
	flow_rays_msg.scale.x = 1;
	flow_rays_msg.scale.y = 1;
	flow_rays_msg.scale.z = 1;
	flow_rays_msg.color.a = 0.3;
	flow_rays_msg.color.r = 0.0;
	flow_rays_msg.color.g = 1.0;
	flow_rays_msg.color.b = 0.0;

	int num_pts = tails.size();
	double x, y, z;
	//x = odom_msg_out.pose.pose.position.x;
	y = odom_msg_out.pose.pose.position.y;
	z = odom_msg_out.pose.pose.position.z;
	flow_rays_msg.points.resize(3*num_pts);
	for(int i = 0 ; i < num_pts ; i++){
		flow_rays_msg.points[3*i].x = 2;
		flow_rays_msg.points[3*i].y = y;
		flow_rays_msg.points[3*i].z = z;
		flow_rays_msg.points[3*i+1].x = tails[i].x;
		flow_rays_msg.points[3*i+1].y = tails[i].y;
		flow_rays_msg.points[3*i+1].z = tails[i].z;
		flow_rays_msg.points[3*i+2].x = tips[i].x;
		flow_rays_msg.points[3*i+2].y = tips[i].y;
		flow_rays_msg.points[3*i+2].z = tips[i].z;
	}
	flow_rays_publ.publish(flow_rays_msg);


}

void publish_odom(){
	static int seq = 0;
	odom_msg_out = utils::trans::se32odom(estm_pose);
	odom_msg_out.header.seq = seq++;
	odom_msg_out.header.stamp = ros::Time::now();
	odom_msg_out.header.frame_id = "world";
	odom_publ.publish(odom_msg_out);
}

void publish_map(){
	octomap_msg.header.frame_id = "world";
	octomap_msg.header.stamp    = ros::Time::now();
	octomap_msg.header.seq++;
	octomap_msgs::binaryMapToMsg(roof_mapper.get_octomap(), octomap_msg);
	map_publ.publish(octomap_msg);
}

