#include <list>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>



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

double refresh_rate;
bool   debug_mode;

ros::Publisher  odom_publ, 
				laser_pc_publ,
				res_rays_publ,
				map_pc_publ,
				flow_rays_publ;
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

string map_path;
pcl::PointCloud<pcl::PointXYZ>::Ptr grid_map;

// TunnelLocalizer class wraps around the range based localizer and
// image based localizer. Through pushing sensor data, corresponding
// localizer is populated with sensor data. This continues until a
// call to pose estimate routine which then flushes saved data.
TunnelLocalizer tunnel_localizer;

sensor_msgs::LaserScan bot_lidar_msg, top_lidar_msg;
sensor_msgs::Image	top_cam_msg, bot_cam_msg, 
					right_cam_msg, left_cam_msg;
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg_in, odom_msg_out;
visualization_msgs::Marker res_rays_msg, flow_rays_msg;

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

void publish_res_rays(const ros::NodeHandle &n);
void publish_flow_rays(const ros::NodeHandle &n);

static Eigen::Vector3d rpy;
static Eigen::Vector4d quat;
static Eigen::Matrix3d dcm;
static Eigen::Matrix4d init_pose;
static Eigen::Matrix4d estm_pose;

int main(int argc, char* argv[]){

	ros::init(argc, argv, "tunnel_localizer");
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

	n.param("top_lidar_calib_file"		, top_lidar_calib_file	 , string("ERROR"));
	n.param("bottom_lidar_calib_file"	, bottom_lidar_calib_file, string("ERROR"));
	n.param("top_cam_calib_file"		, top_cam_calib_file	 , string("ERROR"));
	n.param("right_cam_calib_file"		, right_cam_calib_file	 , string("ERROR"));
	n.param("bottom_cam_calib_file"		, bottom_cam_calib_file	 , string("ERROR"));
	n.param("left_cam_calib_file"		, left_cam_calib_file	 , string("ERROR"));
	
	n.param("map_path", map_path, string(""));	
	
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


	ROS_INFO(" ---------- TUNNEL LOCALIZER ------------");
	ROS_INFO("[refresh_rate] ---------- : [%.3f]", refresh_rate);
	ROS_INFO("[debug_mode] ------------ : [%s]", debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[map_path] -------------- : [%s]", map_path.c_str());
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
	//top_lidar_calib_params.print();
	
	bottom_lidar_calib_params.load(bottom_lidar_calib_file);
	//ROS_INFO("Bottom Lidar Calib Params :");
	//bottom_lidar_calib_params.print();

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
	
	pcl::PointCloud<pcl::PointXYZ> temp_grid_map;
	grid_map = temp_grid_map.makeShared();
	// Load the map from the given '.pcd' file path.
	//if(pcl::io::loadPLYFile<pcl::PointXYZ> (map_path.c_str(), *grid_map) == -1){
	if(pcl::io::loadPCDFile<pcl::PointXYZ> (map_path.c_str(), *grid_map) == -1){
		ROS_ERROR ("Couldn't read file map file from the path : %s", map_path.c_str());
	}

	tunnel_localizer.set_map(grid_map);
}

void setup_messaging_interface(ros::NodeHandle &n)
{
	// A proper estimator node is supposed to take "only sensor inputs" and
	// "output an estimate for X". This node's inputs are IMU, 2 x Scan and 
	// 4 x Cams. Its output is the relative pose of the robot w.r.t. to the
	// given map (if any).
	odom_publ        = n.advertise<nav_msgs::Odometry>("odom", 10);
	imu_subs         = n.subscribe("imu", 10    , imu_callback, ros::TransportHints().tcpNoDelay()); 
	top_lidar_subs   = n.subscribe("scan/top"   , 10, top_lidar_callback, ros::TransportHints().tcpNoDelay()); 
	bot_lidar_subs   = n.subscribe("scan/bottom", 10, bot_lidar_callback, ros::TransportHints().tcpNoDelay()); 
	top_cam_subs	 = n.subscribe("cam/top"	, 10, top_cam_callback		, ros::TransportHints().tcpNoDelay()); 
	bottom_cam_subs  = n.subscribe("cam/bottom"	, 10, bottom_cam_callback	, ros::TransportHints().tcpNoDelay()); 
	right_cam_subs   = n.subscribe("cam/right"	, 10, right_cam_callback	, ros::TransportHints().tcpNoDelay()); 
	left_cam_subs    = n.subscribe("cam/left"	, 10, left_cam_callback		, ros::TransportHints().tcpNoDelay()); 
	// ### Add RGBD subscriber
	
	// Below are the 'debug' outputs. These (in/out)puts are not inevitable 
	// for the proper execution of the node. However for visualization and
	// debugging purposes, they provide convenience.
	laser_pc_publ	= n.advertise<sensor_msgs::PointCloud2>("debug/pc/aligned_sensor_data", 10);
	res_rays_publ   = n.advertise<visualization_msgs::Marker>("debug/residual_rays", 10);
	flow_rays_publ  = n.advertise<visualization_msgs::Marker>("debug/flow_rays", 10);
	// The current implementation loads the map from a '.pcd' file.
	// In the following versions, we may swich to listening to octomap messages.
	map_pc_publ		= n.advertise<pcl::PointCloud<pcl::PointXYZ> >("debug/pc/map", 10);
}

void loop(const ros::NodeHandle &n)
{
	int map_publ_flag = 0;
	ros::Rate r(refresh_rate);

	while (n.ok())
	{
		r.sleep();
		ros::spinOnce();

		geometry_msgs::Quaternion odom_quat = odom_msg_in.pose.pose.orientation;
		geometry_msgs::Quaternion  imu_quat =  imu_msg.orientation;
		// Check if we got an odometry message. Then assign initial pose for the estimator
		// as the odometry pose.
		
		init_pose = Eigen::Matrix4d::Identity();
		init_pose(0, 3) = 2;
		// Check if either odom_msg_in or imu_msg provide orientation information. If none
		// are available, postpone pose estimation.
		if(odom_quat.w != 0 || odom_quat.x != 0 || odom_quat.y != 0 || odom_quat.z != 0){
			quat(0) = odom_quat.w;
			quat(1) = odom_quat.x;
			quat(2) = odom_quat.y;
			quat(3) = odom_quat.z;
			dcm = utils::quat2dcm(quat);
			dcm = utils::cancel_yaw(dcm);
			init_pose.topLeftCorner(3, 3) = dcm;
			init_pose(0, 3) = odom_msg_in.pose.pose.position.x;
			init_pose(1, 3) = odom_msg_in.pose.pose.position.y;
			init_pose(2, 3) = odom_msg_in.pose.pose.position.z;
		} else if(imu_quat.w != 0 || imu_quat.x != 0 || imu_quat.y != 0 || imu_quat.z != 0){
			quat(0) = imu_quat.w;
			quat(1) = imu_quat.x;
			quat(2) = imu_quat.y;
			quat(3) = imu_quat.z;
			dcm = utils::quat2dcm(quat);
			dcm = utils::cancel_yaw(dcm);
			init_pose.topLeftCorner(3, 3) = dcm.transpose();
			init_pose(0, 3) = 10;
			init_pose(1, 3) = 0;
			init_pose(2, 3) = 1.5;

		} else
			continue;
		
		// Push lidar data 
		int num_lidar_data = 0;
    if(top_lidar_msg.ranges.size() * top_lidar_msg.header.stamp.sec * top_lidar_msg.header.stamp.nsec != 0){
		    tunnel_localizer.push_lidar_data(top_lidar_msg, top_lidar_calib_params, true);
		    top_lidar_msg.header.stamp.sec = 0;
		    num_lidar_data++;
		}
    if(bot_lidar_msg.ranges.size() * bot_lidar_msg.header.stamp.sec * bot_lidar_msg.header.stamp.nsec != 0){
    		tunnel_localizer.push_lidar_data(bot_lidar_msg, bottom_lidar_calib_params);
		    bot_lidar_msg.header.stamp.sec = 0;
		    num_lidar_data++;
    }
    if(right_cam_msg.data.size() * right_cam_msg.header.stamp.sec * right_cam_msg.header.stamp.nsec != 0){
    		tunnel_localizer.push_camera_data(right_cam_msg, right_cam_calib_params);
    		right_cam_msg.header.stamp.sec = 0;
    }
    /*
    if(top_cam_msg.data.size() * top_cam_msg.header.stamp.sec * top_cam_msg.header.stamp.nsec != 0){
    		tunnel_localizer.push_camera_data(top_cam_msg  , top_cam_calib_params);
    		top_cam_msg.header.stamp.sec = 0;
    }
    if(bot_cam_msg.data.size() * bot_cam_msg.header.stamp.sec * bot_cam_msg.header.stamp.nsec != 0){
    		tunnel_localizer.push_camera_data(bot_cam_msg  , bottom_cam_calib_params);
    		bot_cam_msg.header.sec = 0;
    }
    */
    if(left_cam_msg.data.size() * left_cam_msg.header.stamp.sec * left_cam_msg.header.stamp.nsec != 0){
    		tunnel_localizer.push_camera_data(left_cam_msg , left_cam_calib_params);
    		left_cam_msg.header.stamp.sec = 0;
    }

    if(num_lidar_data == 2){
    		tunnel_localizer.estimate_pose(init_pose);
    		tunnel_localizer.get_pose(estm_pose);
    		Eigen::Matrix6d cov;
    		tunnel_localizer.get_covariance(cov);
    } else 
      continue;
		
		odom_msg_out.header.seq++;
		odom_msg_out.header.stamp = ros::Time::now();
		odom_msg_out.header.frame_id = "map";
		odom_msg_out.pose.pose.position.x = estm_pose(0, 3) + 2;
		odom_msg_out.pose.pose.position.y = estm_pose(1, 3);
		odom_msg_out.pose.pose.position.z = estm_pose(2, 3);
		dcm  = estm_pose.topLeftCorner(3, 3);
		quat = utils::dcm2quat(dcm);
		odom_msg_out.pose.pose.orientation.w = quat(0);
		odom_msg_out.pose.pose.orientation.x = quat(1);
		odom_msg_out.pose.pose.orientation.y = quat(2);
		odom_msg_out.pose.pose.orientation.z = quat(3);
		odom_publ.publish(odom_msg_out);

    publish_res_rays(n);
		publish_flow_rays(n);

		//cout << "B3" << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr registered_lidar_cloud;
		tunnel_localizer.get_registered_lidar_cloud(registered_lidar_cloud);
		registered_lidar_cloud->header.frame_id = "map";
		registered_lidar_cloud->header.stamp = ros::Time::now().toSec() * 1000;
		for(int i = 0 ; i < (int)registered_lidar_cloud->points.size() ; i++)
		  registered_lidar_cloud->points[i].x += odom_msg_out.pose.pose.position.x - 2;
		laser_pc_publ.publish(*registered_lidar_cloud);

		//cout << "B4" << endl;
		if(--map_publ_flag <= 0){
			grid_map->header.frame_id = "map";
			grid_map->header.stamp = ros::Time::now().toSec() * 1000;
			map_pc_publ.publish(*grid_map);
			map_publ_flag = 200;
		}

			}
}

void imu_callback(const sensor_msgs::Imu &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got IMU Data");

	imu_msg = msg;
}

void bot_lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Bottom Lidar Data");

	bot_lidar_msg = msg;
}
void top_lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Top Lidar Data");

	top_lidar_msg = msg;
}

void top_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Top Camera Data");

	top_cam_msg = msg;
}

void bottom_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Bottom Camera Data");

	bot_cam_msg = msg;
}

void right_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Right Camera Data");

	right_cam_msg = msg;
}

void left_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Left Camera Data");

	left_cam_msg = msg;
}

void odom_callback(const nav_msgs::Odometry &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Odometry Data");

	odom_msg_in = msg;
}

// This function visualizes the residual vectors after ICP.
// Hence it gives an idea on how well the RangeBasedTunnelLocalizer
// performs.
void publish_res_rays(const ros::NodeHandle &n){
	vector<pcl::PointXYZ> range, map;
	tunnel_localizer.get_range_map_correspondences(range, map);

	res_rays_msg.header.seq++;
	res_rays_msg.header.frame_id = "map";
	res_rays_msg.header.stamp = ros::Time::now();

	res_rays_msg.id = 0;
	res_rays_msg.type = visualization_msgs::Marker::LINE_LIST;
	res_rays_msg.action = visualization_msgs::Marker::ADD;
	res_rays_msg.pose.position.x = odom_msg_out.pose.pose.position.x - 2;
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


void publish_flow_rays(const ros::NodeHandle &n){
	vector<pcl::PointXYZ> tails, pits;
	tunnel_localizer.get_back_projected_features(tails, tips);

	flow_rays_msg.header.seq++;
	flow_rays_msg.header.frame_id = "map";
	flow_rays_msg.header.stamp = ros::Time::now();

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
