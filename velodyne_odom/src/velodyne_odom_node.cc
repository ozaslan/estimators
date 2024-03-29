#include "velodyne_odom.hh"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "utils.hh"

//#include "pose_graph_visualization/PoseGraph.h"
/*
 */

#ifndef M_PI
#define M_PI 3.14159265359  
#endif

ros::Subscriber velodyne_subs;
ros::Subscriber realsense_subs;
ros::Subscriber imu_subs;
ros::Subscriber odom_subs;
ros::Subscriber range_subs;
ros::Publisher  map_publ;
ros::Publisher  local_map_publ;
ros::Publisher  odom_publ;
ros::Publisher  aligned_pc_publ;
ros::Publisher  keyframe_pc_publ;
//ros::Publisher  pose_graph_publ;

vector<sensor_msgs::Imu> imu_msgs;
vector<sensor_msgs::PointCloud2> realsense_msgs;
nav_msgs::Odometry odom_msg, init_odom_msg;
sensor_msgs::PointCloud2 map_msg, local_map_msg, velodyne_msg, aligned_pc_msg, keyframe_pc_msg;
vector<sensor_msgs::Range> range_msgs;
//pose_graph_visualization::PoseGraph pose_graph_msg;

void process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
void velodyne_callback(const sensor_msgs::PointCloud2 &msg);
void realsense_callback(const sensor_msgs::PointCloud2 &msg);
void imu_callback(const sensor_msgs::Imu &msg);
void odom_callback(const nav_msgs::Odometry &msg);
void range_callback(const sensor_msgs::Range &msg);
int  publish_map();
int  publish_local_map();
int  publish_odom();
//int  publish_pose_graph();
//int  publish_range();

double imu_velo_time_offset = 0.10; // sec : imu leading
double range_velo_time_offset = 0.0; // sec : range leading
double realsense_velo_time_offset = 0.0; // sec : realsense leading


bool debug_mode;
bool exclude_x_pos = false;
bool temporal_kf_mode = false;
double temporal_kf_interval = 1.5;

VelodyneOdom velodyne_odom;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "velodyne_odom_node");
	ros::NodeHandle n("~");

	process_inputs(n);	
	setup_messaging_interface(n);

	ros::spin();

	return 0;
}

double yaw, pitch, roll;

void process_inputs(const ros::NodeHandle &n)
{
	n.param("debug_mode", debug_mode, true);
	
	VelodyneOdom::VelodyneOdomParams params;
	n.param("exclude_x_pos", exclude_x_pos, false);
	n.param("temporal_kf_mode", temporal_kf_mode, false);
	n.param("temporal_kf_interval", temporal_kf_interval, 1.5);
	n.param("roll", roll, 0.0);
	n.param("pitch", pitch, 0.0);
	n.param("yaw", yaw, 0.0);
	n.param("voxel_leaf_size/x", params.voxel_leaf_size[0], 0.15);
	n.param("voxel_leaf_size/y", params.voxel_leaf_size[1], 0.15);
	n.param("voxel_leaf_size/z", params.voxel_leaf_size[2], 0.15);
	n.param("ndt/eps", params.ndt_eps, 0.01);
	n.param("ndt/step_size", params.ndt_step_size, 0.1);
	n.param("ndt/res", params.ndt_res, 1.0);
	n.param("ndt/max_iter", params.ndt_max_iter, 7);
	n.param("ndt/fitness_score_thres", params.ndt_fitness_score_thres, 1200.0);
	n.param("batch_ndt/eps", params.batch_ndt_eps, 0.01);
	n.param("batch_ndt/step_size", params.batch_ndt_step_size, 0.1);
	n.param("batch_ndt/res", params.batch_ndt_res, 1.0);
	n.param("batch_ndt/max_iter", params.batch_ndt_max_iter, 10);
	n.param("local_map_max_points", params.local_map_max_points, 7000);
	n.param("local_map_dims/x", params.local_map_dims[0], 40.0);
	n.param("local_map_dims/y", params.local_map_dims[1], 40.0);
	n.param("local_map_dims/z", params.local_map_dims[2], 40.0);
	n.param("init_keyframe_thres/trans", params.init_keyframe_trans_thres, 1.0);
	n.param("init_keyframe_thres/rot"  , params.init_keyframe_rot_thres, DEG2RAD(10));
	n.param("method", params.method, string("NDT"));
  n.param("icp/use_reciprocal_corr", params.icp_use_reciprocal_corr, true);
  n.param("icp/max_corr_dist", params.icp_max_corr_dist, 1.5);
  n.param("icp/max_iter", params.icp_max_iter, 50);
  n.param("icp/trans_eps", params.icp_trans_eps, 1e-8);
  n.param("icp/euc_fitness_eps", params.icp_euc_fitness_eps, 1.0);
  n.param("nicp/use_reciprocal_corr", params.nicp_use_reciprocal_corr, true);
  n.param("nicp/max_corr_dist", params.nicp_max_corr_dist, 1.5);
  n.param("nicp/max_iter", params.nicp_max_iter, 50);
  n.param("nicp/trans_eps", params.nicp_trans_eps, 1e-8);
  n.param("nicp/euc_fitness_eps", params.nicp_euc_fitness_eps, 1.0);
  n.param("gicp/max_iter", params.gicp_max_iter, 50);
  n.param("gicp/rot_eps", params.gicp_rot_eps, 0.001);
  n.param("gicp/corr_randomness", params.gicp_corr_randomness, 3);
  n.param("gicp/max_corr_dist", params.gicp_max_corr_dist, 1.5);
  n.param("imu_velo_time_offset", imu_velo_time_offset, 0.3);

	ROS_INFO(" ---------- VELODYNE ODOM NODE ------------");
	ROS_INFO("[debug_mode] ---------- : [%s]"      , debug_mode ? "TRUE" : "FALSE");
	params.print();
	ROS_INFO(" ------------------------------------------");

	velodyne_odom = VelodyneOdom(params);

  velodyne_odom.set_temporal_kf_mode(temporal_kf_mode);
  velodyne_odom.set_temporal_kf_interval(temporal_kf_interval);
}

int setup_messaging_interface(ros::NodeHandle &n)
{
	if(debug_mode)	{
		ROS_INFO("VELODYNE ODOM NODE : setting up messaging interface.");
		ROS_INFO(" --- Listening  : ~velodyne_points");
		ROS_INFO(" --- Listening  : ~imu_raw");
		ROS_INFO(" --- Listening  : ~init_odom");
		ROS_INFO(" --- Listening  : ~range");
		ROS_INFO(" --- Publishing : ~map");
		ROS_INFO(" --- Publishing : ~odom");
		ROS_INFO(" --- Publishing : ~aligned_pc");
		ROS_INFO(" --- Publishing : ~keyframe_pc");
		//ROS_INFO(" --- Publishing : ~pose_graph");
	}

	imu_subs		= n.subscribe("imu", 10, imu_callback, ros::TransportHints().tcpNoDelay());
	range_subs		= n.subscribe("range", 10, range_callback, ros::TransportHints().tcpNoDelay());
	odom_subs		= n.subscribe("init_odom", 10, odom_callback, ros::TransportHints().tcpNoDelay());
	velodyne_subs	= n.subscribe("velodyne_points", 10, velodyne_callback, ros::TransportHints().tcpNoDelay());
	realsense_subs	= n.subscribe("realsense_points", 10, realsense_callback, ros::TransportHints().tcpNoDelay());
	map_publ		= n.advertise<sensor_msgs::PointCloud2>("map", 10);
	local_map_publ	= n.advertise<sensor_msgs::PointCloud2>("local_map", 10);
	aligned_pc_publ	 = n.advertise<sensor_msgs::PointCloud2>("aligned_pc", 10);
	keyframe_pc_publ = n.advertise<sensor_msgs::PointCloud2>("keyframe_pc", 10);
	//pose_graph_publ = n.advertise<pose_graph_visualization::PoseGraph>("pose_graph", 10);
	odom_publ		= n.advertise<nav_msgs::Odometry>("odom", 10);

	return 0;
}

int publish_odom()
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Published odometry to ~odom");

	static int seq = 0;

	Eigen::Matrix4d pose = velodyne_odom.get_pose();

  if(!range_msgs.empty())
  if( range_msgs.back().range >= range_msgs.back().min_range && 
      range_msgs.back().range <= range_msgs.back().max_range)
      pose(2, 3) =  range_msgs.back().range * pose(2, 2);
 
	odom_msg = utils::trans::se32odom(pose);

	odom_msg.header.seq = seq++;
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.header.frame_id = "world";

	Eigen::Matrix6d cov = velodyne_odom.get_covariance();
	for(int r = 0 ; r < 6 ; r++)
		for(int c = 0 ; c < 6 ; c++)
			odom_msg.pose.covariance[6 * r + c] = cov(r, c);

  if(exclude_x_pos == true){
    odom_msg.pose.pose.position.x = 0;
    odom_msg.twist.twist.linear.x = 0;
  }


	odom_publ.publish(odom_msg);

	return 0;
}

int publish_aligned_pc()
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Published aligned_pc PC to ~aligned_pc");

	static int seq = 0;

	pcl::toROSMsg(*velodyne_odom.get_aligned_pc(), aligned_pc_msg);

	aligned_pc_msg.header.seq = seq++;
	aligned_pc_msg.header.stamp = ros::Time::now();
	aligned_pc_msg.header.frame_id = "world";
	aligned_pc_publ.publish(aligned_pc_msg);

	return 0;
}

int publish_keyframe_pc()
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Published keyframe_pc PC to ~keyframe_pc");

	static int seq = 0;

  //cout << "HERE C0" << endl; fflush(NULL);
	pcl::toROSMsg(*velodyne_odom.get_keyframe_pc(), keyframe_pc_msg);

	keyframe_pc_msg.header.seq = seq++;
	keyframe_pc_msg.header.stamp = ros::Time::now();
	keyframe_pc_msg.header.frame_id = "world";
	keyframe_pc_publ.publish(keyframe_pc_msg);

  //cout << "HERE C1" << endl; fflush(NULL);
	return 0;
}

int publish_map()
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Published map PC to ~map");

	static int seq = 0;

  //cout << "HERE C2" << endl; fflush(NULL);
	pcl::toROSMsg(*velodyne_odom.get_map(), map_msg);

	map_msg.header.seq = seq++;
	map_msg.header.stamp = ros::Time::now();
	map_msg.header.frame_id = "world";
	map_publ.publish(map_msg);

  //cout << "HERE C3" << endl; fflush(NULL);
	return 0;
}

int publish_local_map()
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Published local_map PC to ~local_map");

	static int seq = 0;

  //cout << "HERE C4" << endl; fflush(NULL);
	pcl::toROSMsg(*velodyne_odom.get_local_map(), local_map_msg);

	local_map_msg.header.seq = seq++;
	local_map_msg.header.stamp = ros::Time::now();
	local_map_msg.header.frame_id = "world";
	local_map_publ.publish(local_map_msg);

  //cout << "HERE C5" << endl; fflush(NULL);
	return 0;
}

/*
int publish_pose_graph(){

  const vector<pair<int, int> > pose_graph_edges = velodyne_odom.get_keyframe_connectivity();
  const vector<Eigen::Matrix4d> keyframe_poses   = velodyne_odom.get_keyframe_poses();

  pose_graph_msg.poses.resize(keyframe_poses.size());
  pose_graph_msg.edges.resize(2 * pose_graph_edges.size());

  pose_graph_msg.header.seq++;
  pose_graph_msg.header.stamp = ros::Time::now();
  pose_graph_msg.header.frame_id = "world";

	for(int i = 0 ; i < (int)keyframe_poses.size() ; i++){
		pose_graph_msg.poses[i] = utils::trans::se32odom(keyframe_poses[i]).pose.pose;
	}
 
	for(int i = 0 ; i < (int)pose_graph_edges.size() ; i++){
    pose_graph_msg.edges[2 * i + 0] = pose_graph_edges[i].first;
    pose_graph_msg.edges[2 * i + 1] = pose_graph_edges[i].second;
  }

  pose_graph_publ.publish(pose_graph_msg);

  return 0;
}
*/
void imu_callback(const sensor_msgs::Imu &msg)
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Got IMU message!");

	imu_msgs.push_back(msg);
}

void range_callback(const sensor_msgs::Range &msg)
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Got Range message!");

	range_msgs.push_back(msg);
}

void odom_callback(const nav_msgs::Odometry &msg)
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Got Odometry message!");

	if(fabs(msg.twist.twist.linear.x) > 20 ||
	   fabs(msg.twist.twist.linear.y) > 20 ||
	   fabs(msg.twist.twist.linear.z) > 20){
	} else {
		init_odom_msg = msg;
	}
}

void realsense_callback(const sensor_msgs::PointCloud2 &msg){

  if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Got Realsense message!");

  realsense_msgs.push_back(msg);
}

void velodyne_callback(const sensor_msgs::PointCloud2 &msg)
{
	static Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
	static Eigen::Matrix3d prev_imu_dcm, dR_imu;

	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Got Velodyne message!");

  //cout << "HERE C6" << endl; fflush(NULL);
	if(imu_msgs.size() == 0)
		return;

	velodyne_msg = msg;

  //cout << "HERE C7" << endl; fflush(NULL);

  Eigen::Matrix4f velodyne_to_body_tf = Eigen::Matrix4f::Identity();
  velodyne_to_body_tf(2, 3) = 0.15; 
  pcl_ros::transformPointCloud(velodyne_to_body_tf, msg, velodyne_msg);

  //cout << "HERE C8" << endl; fflush(NULL);
  // ------------ Pick the closest IMU msg in time ---------------------------------------------------------- //

	double best_time_offset = 9999;
	int best_time_offset_ind = -1;
	for(int i = 0 ; i < (int)imu_msgs.size() ; i++){
		double dt = std::fabs((imu_msgs[i].header.stamp - velodyne_msg.header.stamp).toSec() + imu_velo_time_offset); 
		if(best_time_offset > dt){
			best_time_offset = dt;
			best_time_offset_ind = i;
		}
	}

  //cout << "HERE C9" << endl; fflush(NULL);
  //cout << "HERE 2" << endl; fflush(NULL);

	if(best_time_offset_ind >= 0){
		Eigen::Vector3d rpy_imu = utils::trans::dcm2rpy(utils::trans::imu2dcm(imu_msgs[best_time_offset_ind]));
		Eigen::Vector3d rpy_init_pose = utils::trans::dcm2rpy(init_pose.topLeftCorner<3, 3>());
    static double prev_yaw = rpy_imu(2);
		rpy_init_pose(0) = rpy_imu(0);
		rpy_init_pose(1) = rpy_imu(1);
    rpy_init_pose(2) += rpy_imu(2) - prev_yaw;
    prev_yaw = rpy_imu(2);
		init_pose.topLeftCorner<3, 3>() = utils::trans::rpy2dcm(rpy_init_pose);
	}

  //cout << "HERE 3" << endl; fflush(NULL);

  //cout << "HERE C10" << endl; fflush(NULL);
  if(best_time_offset_ind >= 0 )
		prev_imu_dcm = utils::trans::imu2dcm(imu_msgs[best_time_offset_ind]);
	if(best_time_offset_ind >= 1 )
		imu_msgs.erase(imu_msgs.begin(), imu_msgs.begin() + best_time_offset_ind - 1);

 	if(init_odom_msg.pose.covariance[0] != 0){
		//init_pose(0, 3) = init_odom_msg.pose.pose.position.x;
		//init_pose(1, 3) = init_odom_msg.pose.pose.position.y;
		//init_pose(2, 3) = init_odom_msg.pose.pose.position.z;
    init_pose = utils::trans::odom2se3(init_odom_msg);
	}

  //cout << "HERE 4" << endl; fflush(NULL);
  // ------------ Pick the closest Range msg in time -------------------------------------------------------- //
  best_time_offset = 9999;
	best_time_offset_ind = -1;
	for(int i = 0 ; i < (int)range_msgs.size() ; i++){
		double dt = std::fabs((range_msgs[i].header.stamp - velodyne_msg.header.stamp).toSec() + range_velo_time_offset); 
		if(best_time_offset > dt){
			best_time_offset = dt;
			best_time_offset_ind = i;
		}
	}

  //cout << "HERE C11" << endl; fflush(NULL);
  //cout << "HERE 5" << endl; fflush(NULL);

  if(best_time_offset_ind >= 0){
    double range = range_msgs[best_time_offset_ind].range;
    double min_range = range_msgs[best_time_offset_ind].min_range;
    double max_range = range_msgs[best_time_offset_ind].max_range;
    if(range >= min_range && range <= max_range)
      init_pose(2, 3) =  range * init_pose(2, 2);
  }

  //cout << "HERE 6" << endl; fflush(NULL);

	if(best_time_offset_ind >= 1 )
		range_msgs.erase(range_msgs.begin(), range_msgs.begin() + best_time_offset_ind - 1);

  //cout << "Range reading is : " << range_msg.range << endl;

  // ------------ Pick the closest Realsense msg in time ---------------------------------------------------- //
  best_time_offset = 9999;
	best_time_offset_ind = -1;
	for(int i = 0 ; i < (int)realsense_msgs.size() ; i++){
		double dt = std::fabs((realsense_msgs[i].header.stamp - velodyne_msg.header.stamp).toSec() + realsense_velo_time_offset); 
		if(best_time_offset > dt){
			best_time_offset = dt;
			best_time_offset_ind = i;
		}
	}

  //cout << "HERE 7 " << best_time_offset_ind << endl; fflush(NULL);

  //cout << "HERE C12" << endl; fflush(NULL);
  sensor_msgs::PointCloud2 realsense_pc_out;
  if(best_time_offset_ind >= 0){
    Eigen::Matrix4f realsense_to_body_tf = Eigen::Matrix4f::Identity();
    realsense_to_body_tf(0, 3) =  0.05; 
    realsense_to_body_tf(2, 3) = -0.15; 
    Eigen::Vector3d rpy(roll, pitch, yaw);
    realsense_to_body_tf.topLeftCorner<3, 3>() = utils::trans::rpy2dcm(rpy).cast<float>();
    pcl_ros::transformPointCloud(realsense_to_body_tf, realsense_msgs[best_time_offset_ind], realsense_pc_out);
  }

	if(best_time_offset_ind >= 1 )
		realsense_msgs.erase(realsense_msgs.begin(), realsense_msgs.begin() + best_time_offset_ind - 1);

  //cout << "HERE 8" << endl; fflush(NULL);
  // -------------------------------------------------------------------------------------------------------- //

  if(best_time_offset_ind >= 0)
  	velodyne_odom.push_pc(velodyne_msg, realsense_pc_out);
  else
  	velodyne_odom.push_pc(velodyne_msg);
	if(velodyne_odom.align(init_pose) == 0 || true){
		init_pose = velodyne_odom.get_pose();
		publish_odom();
	  publish_aligned_pc();
	}

	publish_map();
	publish_local_map();
	publish_keyframe_pc();
	//publish_pose_graph();
  //publish_range();
	
  //cout << "HERE C13" << endl; fflush(NULL);
	//cout << "END OF VELODYNE CALLBACK" << endl; fflush(NULL);
}

