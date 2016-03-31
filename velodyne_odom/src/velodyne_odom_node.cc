#include "velodyne_odom.hh"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "utils.hh"
/*
*/

ros::Subscriber velodyne_subs;
ros::Subscriber imu_subs;
ros::Publisher  map_publ;
ros::Publisher  odom_publ;
ros::Publisher  aligned_pc_publ;
ros::Publisher  keyframe_pc_publ;
ros::Publisher  keyframe_poses_publ;

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;
sensor_msgs::PointCloud2 map_msg, velodyne_msg, aligned_pc_msg, keyframe_pc_msg;
geometry_msgs::PoseArray keyframe_poses_msg;

void process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
void velodyne_callback(const sensor_msgs::PointCloud2 &msg);
void imu_callback(const sensor_msgs::Imu &msg);
int  publish_map();
int  publish_odom();
int  publish_pose_array();

bool debug_mode;

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

void process_inputs(const ros::NodeHandle &n)
{
	n.param("debug_mode", debug_mode, true);

	//n.param("grid_cols"		  , ofe_params.grid_cols, 29);

	ROS_INFO(" ---------- VELODYNE ODOM NODE ------------");
	ROS_INFO("[debug_mode] ---------- : [%s]"      , debug_mode ? "TRUE" : "FALSE");
	//ROS_INFO("grid_[rows, cols] ----- : [%d, %d]"  , ofe_params.grid_rows, ofe_params.grid_cols);
	ROS_INFO(" ------------------------------------------");
}

int setup_messaging_interface(ros::NodeHandle &n)
{
	if(debug_mode)	{
		ROS_INFO("VELODYNE ODOM NODE : setting up messaging interface.");
		ROS_INFO(" --- Listening  : ~velodyne_points");
		ROS_INFO(" --- Listening  : ~imu_raw");
		ROS_INFO(" --- Publishing : ~map");
		ROS_INFO(" --- Publishing : ~odom");
		ROS_INFO(" --- Publishing : ~aligned_pc");
		ROS_INFO(" --- Publishing : ~keyframe_pc");
		ROS_INFO(" --- Publishing : ~keyframe_poses");
	}
		
	imu_subs		= n.subscribe("imu", 10, imu_callback, ros::TransportHints().tcpNoDelay());
	velodyne_subs	= n.subscribe("velodyne_points", 10, velodyne_callback, ros::TransportHints().tcpNoDelay());
	map_publ		= n.advertise<sensor_msgs::PointCloud2>("map", 10);
	aligned_pc_publ	 = n.advertise<sensor_msgs::PointCloud2>("aligned_pc", 10);
	keyframe_pc_publ = n.advertise<sensor_msgs::PointCloud2>("keyframe_pc", 10);
	keyframe_poses_publ = n.advertise<geometry_msgs::PoseArray>("keyframe_poses", 10);
	odom_publ		= n.advertise<nav_msgs::Odometry>("odom", 10);

	return 0;
}

int publish_odom()
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Published odometry to ~odom");

	static int seq = 0;
	
	Eigen::Matrix4d pose = velodyne_odom.get_pose();

	odom_msg = utils::trans::se32odom(pose);

	odom_msg.header.seq = seq++;
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.header.frame_id = "world";

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

	pcl::toROSMsg(*velodyne_odom.get_keyframe_pc(), keyframe_pc_msg);

	keyframe_pc_msg.header.seq = seq++;
	keyframe_pc_msg.header.stamp = ros::Time::now();
	keyframe_pc_msg.header.frame_id = "world";
	keyframe_pc_publ.publish(keyframe_pc_msg);

	return 0;
}

int  publish_pose_array(){
  if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Published pose_array to ~keyframe_poses");

	static int seq = 0;

  keyframe_poses_msg.header.seq = seq++;
  keyframe_poses_msg.header.stamp = ros::Time::now();
  keyframe_poses_msg.header.frame_id = "world";

  const vector<Eigen::Matrix4d> keyframe_poses = velodyne_odom.get_keyframe_poses();

  keyframe_poses_msg.poses.resize(keyframe_poses.size());

  Eigen::Vector4d quat;

  for(int i = 0 ; i < (int)keyframe_poses.size() ; i++){
    keyframe_poses_msg.poses[i].position.x = keyframe_poses[i](0, 3);
    keyframe_poses_msg.poses[i].position.y = keyframe_poses[i](1, 3);
    keyframe_poses_msg.poses[i].position.z = keyframe_poses[i](2, 3);
    quat = utils::trans::dcm2quat(keyframe_poses[i].topLeftCorner<3, 3>());
    keyframe_poses_msg.poses[i].orientation.w = quat(0);
    keyframe_poses_msg.poses[i].orientation.x = quat(1);
    keyframe_poses_msg.poses[i].orientation.y = quat(2);
    keyframe_poses_msg.poses[i].orientation.z = quat(3);
  }

	keyframe_poses_publ.publish(keyframe_poses_msg);

	return 0;
}


int publish_map()
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Published map PC to ~map");

	static int seq = 0;

	pcl::toROSMsg(*velodyne_odom.get_map(), map_msg);

	map_msg.header.seq = seq++;
	map_msg.header.stamp = ros::Time::now();
	map_msg.header.frame_id = "world";
	map_publ.publish(map_msg);

	return 0;
}

void imu_callback(const sensor_msgs::Imu &msg)
{
	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Got IMU message!");

	imu_msg = msg;
}

void velodyne_callback(const sensor_msgs::PointCloud2 &msg)
{
	static Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
	static Eigen::Matrix3d prev_imu_dcm, dR_imu;

	if(debug_mode)
		ROS_INFO("VELODYNE ODOM NODE : Got Velodyne message!");

	dR_imu = utils::trans::imu2dcm(imu_msg).inverse() * prev_imu_dcm;
	cout << "rpy = " << utils::trans::dcm2rpy(dR_imu) << endl;
	//init_pose.topLeftCorner<3, 3>() = dR_imu * init_pose.topLeftCorner<3, 3>();
	
	velodyne_msg = msg;

	velodyne_odom.push_pc(msg);
	if(velodyne_odom.align(init_pose) == 0)
		init_pose = velodyne_odom.get_pose();

	publish_odom();
	publish_map();
	publish_aligned_pc();
  publish_keyframe_pc();
  publish_pose_array();
	prev_imu_dcm = utils::trans::imu2dcm(imu_msg);
}

