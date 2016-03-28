#include "velodyne_odom.hh"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;
sensor_msgs::PointCloud2 map_msg, velodyne_msg, aligned_pc_msg;

void process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
void velodyne_callback(const sensor_msgs::PointCloud2 &msg);
void imu_callback(const sensor_msgs::Imu &msg);
int  publish_map();
int  publish_odom();

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
	}
		
	imu_subs		= n.subscribe("imu", 10, imu_callback, ros::TransportHints().tcpNoDelay());
	velodyne_subs	= n.subscribe("velodyne_points", 10, velodyne_callback, ros::TransportHints().tcpNoDelay());
	map_publ		= n.advertise<sensor_msgs::PointCloud2>("map", 10);
	aligned_pc_publ	= n.advertise<sensor_msgs::PointCloud2>("aligned_pc", 10);
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

	prev_imu_dcm = utils::trans::imu2dcm(imu_msg);
}

