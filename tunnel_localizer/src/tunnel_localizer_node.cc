#include "tunnel_localizer_node.hh"

bool got_odom_msg = false;
double x_pos_estm = 7;

int main(int argc, char* argv[]){

	ros::init(argc, argv, "tunnel_localizer_node");
	ros::NodeHandle n("~");

	process_inputs(n);
	setup_messaging_interface(n);
	loop();	

	return 1;
}

void process_inputs(const ros::NodeHandle &n)
{
	// Query for parameter file paths from the parameter server.
	string top_cam_calib_file,
		   right_cam_calib_file,
		   bottom_cam_calib_file,
		   left_cam_calib_file,
		   top_lidar_calib_file,
		   bottom_lidar_calib_file,
		   map_path;

	n.param("refresh_rate", refresh_rate, 40.0);
	n.param("debug_mode", debug_mode, false);

	n.param("top_lidar_calib_file"		, top_lidar_calib_file	    , string("ERROR"));
	n.param("bottom_lidar_calib_file"	, bottom_lidar_calib_file   , string("ERROR"));
	n.param("top_cam_calib_file"		  , top_cam_calib_file	      , string("ERROR"));
	n.param("right_cam_calib_file"		, right_cam_calib_file	    , string("ERROR"));
	n.param("bottom_cam_calib_file"		, bottom_cam_calib_file	    , string("ERROR"));
	n.param("left_cam_calib_file"		  , left_cam_calib_file	      , string("ERROR"));
	n.param("median_filter_window"		, median_filter_window      , 5);
	n.param("rate_linearity_window"   , rate_linearity_window     , 15);
	n.param("downsample_thres"			  , downsample_thres		      , 0.03);
	n.param("remove_occlusion_thres"  , remove_occlusion_thres    , 0.5);
	n.param("remove_occlusion_window" , remove_occlusion_window   , 3);
	n.param("cluster_min_skips"			  , cluster_min_skips		      , 3);
	n.param("cluster_range_jump"		  , cluster_range_jump        , 0.50);
	n.param("cluster_min_cluster_size", cluster_min_cluster_size  , 3);
	n.param("linearity_thres"			    , linearity_thres		        , 0.3);

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
	ROS_INFO("[refresh_rate] ----------- : [%.3f]", refresh_rate);
	ROS_INFO("[debug_mode] ------------- : [%s]"  , debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[map_path] --------------- : [%s]"  , map_path.c_str());
	ROS_INFO("[top_lidar_calib_file] --- : [%s]"  , top_lidar_calib_file.c_str());
	ROS_INFO("[bottom_lidar_calib_file]  : [%s]"  , bottom_lidar_calib_file.c_str());
	ROS_INFO("[top_cam_calib_file] ----- : [%s]"  , top_cam_calib_file.c_str());
	ROS_INFO("[right_cam_calib_file] --- : [%s]"  , right_cam_calib_file.c_str());
	ROS_INFO("[bottom_cam_calib_file] -- : [%s]"  , bottom_cam_calib_file.c_str());
	ROS_INFO("[left_cam_calib_file] ---- : [%s]"  , left_cam_calib_file.c_str());
	ROS_INFO("[median_filter_window] --- : [%d]"  , median_filter_window);
	ROS_INFO("[rate_linearity_window] -- : [%.d]" , rate_linearity_window);
	ROS_INFO("[downsample_thres] ------- : [%.3f]", downsample_thres);
	ROS_INFO("[remove_occlusion_thres] - : [%.3f]", remove_occlusion_thres);
	ROS_INFO("[remove_occlusion_window]  : [%d]"  , remove_occlusion_window);
	ROS_INFO("[cluster_min_skips] ------ : [%d]"  , cluster_min_skips);
	ROS_INFO("[cluster_range_jump] ----- : [%.3f]", cluster_range_jump);
	ROS_INFO("[cluster_min_cluster_size] : [%d]"  , cluster_min_cluster_size);
	ROS_INFO("[linearity_thres] -------- : [%.3f]", linearity_thres);

	ROS_INFO(" ----------------------------------------");

	// Load calibration data for each sensor. 
	// ### (to be implemented) In case of load failure, 'is_valid'
	// flag is set to 'false'.

	top_lidar_calib_params.load(top_lidar_calib_file);
	ROS_INFO("Top Lidar Calib Params :");
	top_lidar_calib_params.print();

	bottom_lidar_calib_params.load(bottom_lidar_calib_file);
	ROS_INFO("Bottom Lidar Calib Params :");
	bottom_lidar_calib_params.print();

	//top_cam_calib_params.load(top_cam_calib_file);
	//ROS_INFO("Top Cam Calib Params :");
	//top_cam_calib_params.print();

	right_cam_calib_params.load(right_cam_calib_file);
	ROS_INFO("Right Cam Calib Params :");
	right_cam_calib_params.print();

	//bottom_cam_calib_params.load(bottom_cam_calib_file);
	//ROS_INFO("Bottom Cam Calib Params :");
	//bottom_cam_calib_params.print();

	left_cam_calib_params.load(left_cam_calib_file);
	ROS_INFO("Left Cam Calib Params :");
	left_cam_calib_params.print();

	pcl::PointCloud<pcl::PointXYZ> temp_grid_map;
	grid_map = temp_grid_map.makeShared();
	// Load the map from the given '.pcd' file path.
	//if(pcl::io::loadPLYFile<pcl::PointXYZ> (map_path.c_str(), *grid_map) == -1){
	ROS_INFO("... Loading map from '%s'", map_path.c_str());
	if(pcl::io::loadPCDFile<pcl::PointXYZ> (map_path.c_str(), *grid_map) == -1){
		ROS_ERROR ("Couldn't read file map file from the path : %s", map_path.c_str());
	}
	ROS_INFO("... Successfully loaded the map!");

	tunnel_localizer.set_map(grid_map);
}

void setup_messaging_interface(ros::NodeHandle &n)
{
	// A proper estimator node is supposed to take "only sensor inputs" and
	// "output an estimate for X". This node's inputs are IMU, 2 x Scan and 
	// 4 x Cams. Its output is the relative pose of the robot w.r.t. to the
	// given map (if any).
	odom_publ        = n.advertise<nav_msgs::Odometry>("odom_out", 10);
	imu_subs         = n.subscribe("imu"        , 10, imu_callback        , ros::TransportHints().tcpNoDelay()); 
	odom_subs        = n.subscribe("odom_in"    , 10, odom_callback       , ros::TransportHints().tcpNoDelay()); 
	top_lidar_subs   = n.subscribe("scan/top"   , 10, top_lidar_callback  , ros::TransportHints().tcpNoDelay()); 
	bot_lidar_subs   = n.subscribe("scan/bottom", 10, bot_lidar_callback  , ros::TransportHints().tcpNoDelay()); 
	top_cam_subs	   = n.subscribe("cam/top"	  , 10, top_cam_callback		, ros::TransportHints().tcpNoDelay()); 
	bottom_cam_subs  = n.subscribe("cam/bottom"	, 10, bottom_cam_callback	, ros::TransportHints().tcpNoDelay()); 
	right_cam_subs   = n.subscribe("cam/right"	, 10, right_cam_callback	, ros::TransportHints().tcpNoDelay()); 
	left_cam_subs    = n.subscribe("cam/left"	  , 10, left_cam_callback		, ros::TransportHints().tcpNoDelay()); 
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

	image_transport::ImageTransport it(n);
	flow_img_publ = it.advertise("debug/flow_image", 10);
}

void loop()
{
	ros::Rate r(refresh_rate);
	ros::spin();
}

void iterate_estimator(){

	if(is_bottom_lidar_valid == false || is_top_lidar_valid == false)
		return;

	if(is_right_cam_valid == true){
		tunnel_localizer.push_camera_data(right_cam_msg, right_cam_calib_params);
		is_right_cam_valid = false;
	}

	if(is_left_cam_valid == true){
		//tunnel_localizer.push_camera_data( left_cam_msg,  left_cam_calib_params);
		is_left_cam_valid = false;
	}

	top_lidar_proc.transform(Eigen::Matrix4d::Identity(), false);
	bottom_lidar_proc.transform(Eigen::Matrix4d::Identity(), false); 

	tunnel_localizer.push_lidar_data(bottom_lidar_proc, true);
	tunnel_localizer.push_lidar_data(top_lidar_proc, false);

	is_top_lidar_valid    = false;
	is_bottom_lidar_valid = false;

	// ### need to set the map somewhere
	tunnel_localizer.estimate_pose(init_pose);
	tunnel_localizer.get_pose(estm_pose);
	tunnel_localizer.get_covariance(estm_cov);

	x_pos_estm = estm_pose(0, 3);

	//cout << "X_POS_ESTM = " << x_pos_estm << endl;
	//cout << "estm_pose = " << endl << estm_pose << endl;

	publish_odom();
	publish_res_rays();
	publish_laser();
	publish_flow_rays();

	static unsigned seq = 0;
	if(seq++ % 100 == 0)
		publish_map();
}

void odom_callback(const nav_msgs::Odometry &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got ODOM Data");

	init_pose = utils::trans::odom2se3(msg); 
	Eigen::Matrix3d dcm = init_pose.topLeftCorner<3, 3>(); 
	double y = utils::trans::dcm2rpy(dcm)(2);

	dcm = init_pose.topLeftCorner<3, 3>();
	Eigen::Vector3d rpy = utils::trans::dcm2rpy(dcm);
	rpy(2) = y;

	init_pose.topLeftCorner<3, 3>() = utils::trans::rpy2dcm(rpy);

	init_pose(0, 3) = x_pos_estm;
	init_pose(1, 3) = 0;
	init_pose(2, 3) = 0;

	//cout << "ODOM CALLBACK : " << endl;
	//cout << "init_pose = " << endl << init_pose << endl;

	got_odom_msg = true;
}

void imu_callback(const sensor_msgs::Imu &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got IMU Data");

	imu_msg = msg;

	Eigen::Matrix3d	dcm = init_pose.topLeftCorner<3, 3>();
	Eigen::Vector3d rpy = utils::trans::dcm2rpy(dcm);
	double y = rpy(2);
	dcm = utils::trans::imu2dcm(imu_msg, true);
	rpy = utils::trans::dcm2rpy(dcm);
	rpy(2) = y;
	dcm = utils::trans::rpy2dcm(rpy);

	init_pose.topLeftCorner<3, 3>() = dcm;
	if(got_odom_msg == false){
		init_pose(0, 3) = x_pos_estm;
		init_pose(1, 3) = 0;
		init_pose(2, 3) = 0;
		init_pose(3, 3) = 1;
	}

	//cout << "IMU CALLBACK : " << endl;
	//cout << "init_pose = " << endl << init_pose << endl;

}

void bot_lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Bottom Lidar Data");

	bottom_lidar_proc.update_data(msg, bottom_lidar_calib_params);

	bottom_lidar_proc.median_filter(median_filter_window);
	bottom_lidar_proc.cluster(cluster_min_skips, cluster_range_jump, cluster_min_cluster_size);
	bottom_lidar_proc.downsample(downsample_thres);
	bottom_lidar_proc.remove_occlusions(remove_occlusion_thres, remove_occlusion_window);
	bottom_lidar_proc.rate_linearity(rate_linearity_window);
	bottom_lidar_proc.linearity_filter(linearity_thres);
	bottom_lidar_proc.transform(Eigen::Matrix4d::Identity(), true);
	bottom_lidar_proc.estimate_fim();
	is_bottom_lidar_valid = true;

	iterate_estimator();
}

void top_lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Top Lidar Data");

	top_lidar_proc.update_data(msg, top_lidar_calib_params);

	top_lidar_proc.median_filter(median_filter_window);
	top_lidar_proc.cluster(cluster_min_skips, cluster_range_jump, cluster_min_cluster_size);
	top_lidar_proc.downsample(downsample_thres);
	top_lidar_proc.rate_linearity(rate_linearity_window);
	top_lidar_proc.linearity_filter(linearity_thres);
	top_lidar_proc.remove_occlusions(remove_occlusion_thres, remove_occlusion_window);
	top_lidar_proc.transform(Eigen::Matrix4d::Identity(), true);
	top_lidar_proc.estimate_fim();
	is_top_lidar_valid = true;

	iterate_estimator();
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

	is_right_cam_valid = true;
}

void left_cam_callback(const sensor_msgs::Image &msg){
	if(debug_mode)
		ROS_INFO("TUNNEL LOCALIZER : Got Left Camera Data");

	left_cam_msg = msg;

	is_left_cam_valid = true;
}

void publish_odom(){
	static int seq = 0;
	nav_msgs::Odometry odom_msg;
	odom_msg = utils::trans::se32odom(estm_pose, false, estm_cov);

	for(int r = 0 ; r < 4 ; r++){
		for(int c = 0 ; c < 4 ; c++){
			if( estm_pose(r, c) != estm_pose(r, c) )
				return;
		}
	}

	//cout << "PUBLISH ODOM" << endl;
	//cout << "estm_pose = " << endl << estm_pose << endl;

	//odom_msg.pose.pose.position.x = 3;

	odom_msg.header.seq = seq++;
	odom_msg.header.frame_id = "map";
	odom_msg.header.stamp = ros::Time::now();

	// ### add covariance to the message

	odom_publ.publish(odom_msg);
}

// This function visualizes the residual vectors after ICP.
// Hence it gives an idea on how well the RangeBasedTunnelLocalizer
// performs.
void publish_res_rays(){

	vector<pcl::PointXYZ> range, map;
	tunnel_localizer.get_range_map_correspondences(range, map);

	res_rays_msg.header.seq++;
	res_rays_msg.header.frame_id = "map";
	res_rays_msg.header.stamp = ros::Time::now();

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

	vector<cv::Mat> images;
	tunnel_localizer.plot_tracked_features(images);
	/*
	for(int i = 0 ; i < (int)images.size() ; i++){
		char buf[1024];
		sprintf(buf, "Flows %d", i);
		string win_name(buf);
		cv::namedWindow(win_name);
		cv::imshow(win_name, images[i]);
	}
	cv::waitKey(1);*/

	static cv_bridge::CvImage cv_image;
	if(images.size() !=0){
		cv_image.header.seq++;
		cv_image.header.frame_id = "world";
		cv_image.header.stamp    = ros::Time::now();
		if(images[0].channels() == 3)
			cv::cvtColor(images[0], cv_image.image, CV_BGR2GRAY);
		else
			cv_image.image = images[0];
		cv_image.encoding = "mono8";
		flow_img_publ.publish(cv_image.toImageMsg());
	}

	vector<pcl::PointXYZ> tails, pits;
	tunnel_localizer.get_back_projected_features(tails, tips);

	flow_rays_msg.header.seq++;
	flow_rays_msg.header.frame_id = "map";
	flow_rays_msg.header.stamp = ros::Time::now();

	flow_rays_msg.id = 33;
	flow_rays_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
	flow_rays_msg.action = visualization_msgs::Marker::ADD;
	flow_rays_msg.pose.position.x = 0;
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
	flow_rays_msg.points.resize(3*num_pts);
	for(int i = 0 ; i < num_pts ; i++){
		flow_rays_msg.points[3*i].x = estm_pose(0, 3);
		flow_rays_msg.points[3*i].y = estm_pose(1, 3);
		flow_rays_msg.points[3*i].z = estm_pose(2, 3);
		flow_rays_msg.points[3*i+1].x = tails[i].x;
		flow_rays_msg.points[3*i+1].y = tails[i].y;
		flow_rays_msg.points[3*i+1].z = tails[i].z;
		flow_rays_msg.points[3*i+2].x = tips[i].x;
		flow_rays_msg.points[3*i+2].y = tips[i].y;
		flow_rays_msg.points[3*i+2].z = tips[i].z;
	}
	flow_rays_publ.publish(flow_rays_msg);
}

void publish_map(){

	grid_map->header.frame_id = "world";
	grid_map->height = 1;
	grid_map->width = grid_map->points.size();

	grid_map->header.stamp = ros::Time::now().toSec();
	map_pc_publ.publish (grid_map);
}

void publish_laser(){

	pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pc;
	tunnel_localizer.get_registered_lidar_cloud(laser_pc);

	laser_pc->header.frame_id = "world";
	laser_pc->height = 1;
	laser_pc->width = laser_pc->points.size();

	laser_pc->header.stamp = ros::Time::now().toSec();
	laser_pc_publ.publish(laser_pc);

	/*
	   static visualization_msgs::MarkerArray top_marray, bottom_marray;
	   bottom_laser_proc.get_RVIZ_markers(bottom_marray);
	   top_laser_proc.get_RVIZ_markers(   top_marray);
	 */
}
