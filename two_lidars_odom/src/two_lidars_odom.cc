/*
   See the header file for detailed explanations
   of the below functions.
 */

#include "two_lidars_odom.hh"

TwoLidarsOdom::TwoLidarsOdom(int max_iter, double xyz_tol, double rot_tol){
	_max_iter = max_iter;
	_xyz_tol  = xyz_tol;
	_rot_tol  = rot_tol;

	_pose = Eigen::Matrix4d::Identity();
}


Eigen::Matrix4d TwoLidarsOdom::estimate_odometry(const LaserProc bottom_laser_proc[2], 
		const LaserProc    top_laser_proc[2], 
		const Eigen::Matrix4d &init_pose, bool reset_pose, bool switch_order){

	if(reset_pose == true)
		_pose = Eigen::Matrix4d::Identity();

	int zeroth_idx = switch_order ? 1 : 0;
	int first_idx  = switch_order ? 0 : 1;

	vector<vector<int> >	masks(4);
	vector<vector<double> >	ranges(4);
	vector<vector<double> >	linearities(4);
	vector<vector<double> >	ths(4);
	vector<double>	angle_min(4), 
		angle_max(4),
		angle_inc(4);

	// Fetch the data from the laser_proc data structures.
	masks[0] = bottom_laser_proc[zeroth_idx].get_mask();
	masks[1] = bottom_laser_proc[first_idx].get_mask();
	masks[2] = top_laser_proc[zeroth_idx].get_mask();
	masks[3] = top_laser_proc[first_idx].get_mask();

	ranges[0] = bottom_laser_proc[zeroth_idx].get_ranges();
	ranges[1] = bottom_laser_proc[first_idx].get_ranges();
	ranges[2] = top_laser_proc[zeroth_idx].get_ranges();
	ranges[3] = top_laser_proc[first_idx].get_ranges();

	linearities[0] = bottom_laser_proc[zeroth_idx].get_linearity_rates();
	linearities[1] = bottom_laser_proc[first_idx].get_linearity_rates();
	linearities[2] = top_laser_proc[zeroth_idx].get_linearity_rates();
	linearities[3] = top_laser_proc[first_idx].get_linearity_rates();

	angle_min[0] = bottom_laser_proc[zeroth_idx].get_angle_min();
	angle_max[0] = bottom_laser_proc[zeroth_idx].get_angle_max();
	angle_inc[0] = bottom_laser_proc[zeroth_idx].get_angle_increment();
	angle_min[1] = angle_min[0];
	angle_max[1] = angle_max[0];
	angle_inc[1] = angle_inc[0];

	angle_min[2] = top_laser_proc[zeroth_idx].get_angle_min();
	angle_max[2] = top_laser_proc[zeroth_idx].get_angle_max();
	angle_inc[2] = top_laser_proc[zeroth_idx].get_angle_increment();
	angle_min[3] = angle_min[2];
	angle_max[3] = angle_max[2];
	angle_inc[3] = angle_inc[2];

	// Mask the points of low linearity / high cornerness
	// Fill the theta's arrays
	for(int i = 0 ; i < (int)masks.size() ; i++){
		// Allocate memory for range angles
		ths[i].resize(masks[i].size());
		for(int j = 0 ; j < (int)masks[i].size() ; j++){
			if(linearities[i][j] > 0.6)
				masks[i][j] = 0;
			ths[i][j] = angle_min[i] + angle_inc[i] * j;
		}
	}

	// These parameters are basically required for relative poses.
	vector<LidarCalibParams> lidar_params(2);
	lidar_params[0] = bottom_laser_proc[first_idx].get_calib_params();
	lidar_params[1] =    top_laser_proc[first_idx].get_calib_params();
	lidar_params[0].relative_pose.topRightCorner<3, 1>().fill(0);
	lidar_params[1].relative_pose.topRightCorner<3, 1>().fill(0);

	const vector<Eigen::Vector3d>    &top_3d_rays	=	 top_laser_proc[first_idx].get_3d_points(); // 3D rays before transformation
	const vector<Eigen::Vector3d> &bottom_3d_rays	= bottom_laser_proc[first_idx].get_3d_points(); // ''  ''   ''     ''

	/* up until here, I did only initialization ----------------------------------- */
	/* ---------------------------------------------------------------------------- */

	// * transformation from 1st frame to 0th frame
	Eigen::Matrix4d	 dicp = init_pose;
	// change in dicp in one iteration. Use to check for termination.
	Eigen::Matrix4d ddicp;

	// Temporary rotation matrix used for 3D ray projection.
	Eigen::Vector3d csm_init_pose, temp;
	Eigen::Matrix3d T_dcm;
	Eigen::Matrix4d T;
	Eigen::Vector5d csm_result;

	static int index = 1;

	double t_update[2];
	double rot_update[2];

  _max_iter = 2;

	for(int iter = 0 ; iter < _max_iter; iter++){
		cout << "% ------------------------------------------------------%" << endl;
		cout << "iter = " << iter << ";" << endl;
	
		int idx2 = iter % 2;
		int idx4 = 2 * (iter % 2);

		cout << "idx2 = " << idx2 << endl;

		//if(idx2 == 1)
		//	continue;

		// * sensor_relative_pose := sensor -> robot
		Eigen::Matrix4d sensor_relative_pose	 = lidar_params[idx2].relative_pose;
		Eigen::Matrix4d sensor_relative_pose_inv = sensor_relative_pose.inverse();
		const vector<Eigen::Vector3d> &_3d_rays	 = (idx2 == 0) ? bottom_3d_rays : top_3d_rays;

    cout << "dicp[-1] = " << endl << dicp << endl;

		// * transformation from 1st sensor frame to 0th sensor frame
		T = sensor_relative_pose_inv * dicp * sensor_relative_pose;

    cout << "T = " << endl << T << endl;

		// Extract the projection of T on the current sensor frame
		csm_init_pose(0) = T(0, 3); // x
		csm_init_pose(1) = T(1, 3); // y
		csm_init_pose(2) = 0;		// apply the rotation in the below loop instead

		cout << "csm_init_pose = " << csm_init_pose << endl;

		// Rotational component of the transformation free of yaw in the sensor frame
		T_dcm = T.topLeftCorner<3, 3>();

		cout << "T_dcm = " << endl << T_dcm << endl;

		// Apply the initial transformation on the lidar data 
		// and re-calculate the angles
		for(int i = 0 ; i < (int)_3d_rays.size() ; i++){
			temp = T_dcm * _3d_rays[i];
			temp(2) = 0;
			ranges[idx4 + 1][i] = temp.norm();
			ths[idx4 + 1][i] = atan2(temp(1), temp(0));
		}

		// Find 2D transformations with CSM
		/*
		   Eigen::Vector5d register_scan(vector<double> &ranges1, vector<int> &mask1, vector<double> &ths1,
		   vector<double> &ranges2, vector<int> &mask2, vector<double> &ths2,
		   const Eigen::Vector3d &init_pose = Eigen::Vector3d::Zero(),
		   bool  recover_from_error = false, 
		   double  max_angular_correction_deg = 20,
		   double  max_linear_correction = .5,
		   int   max_iterations = 1000, 
		   double  epsilon_xy = 0.001,
		   double  epsilon_theta = 0.001,
		   double  max_correspondence_dist =  2,
		   double  sigma = 0.01,
		   bool  use_corr_tricks = false,
		   bool  restart = true,
		   double  restart_threshold_mean_error = 0.05,
		   double  restart_dt = 0.05,
		   double  restart_dtheta = 5 * 0.0261799);
		*/

		csm_result = utils::laser::register_scan(ranges[idx4], masks[idx4], ths[idx4], ranges[idx4 + 1], masks[idx4 + 1], ths[idx4 + 1], 
				csm_init_pose, false, 20, 0.50, 133, 0.001, 0.001, 0.55, 0.01, true, true); 

		cout << "csm_result = " << csm_result << endl;

		if(idx2 == 1){
			//csm_result(0) = .3;
		}

		if(csm_result(3) == 0){
			cout << "invalid CSM" << endl;
			cout << csm_result(4) << endl;
			continue;
		}

		Eigen::Matrix4d csm_init_pose_se3 = Eigen::Matrix4d::Identity();
		csm_init_pose_se3.topLeftCorner<3, 3>() = utils::trans::yaw2dcm(csm_init_pose(2));
		csm_init_pose_se3(0, 3)	= csm_init_pose(0);
		csm_init_pose_se3(1, 3)	= csm_init_pose(1);
		//csm_init_pose_se3(2, 3)	= T(2, 3);

		// * trans from 1st sensor frame to 0th sensor frame
		ddicp = Eigen::Matrix4d::Identity();
		ddicp.topLeftCorner<3, 3>() = utils::trans::yaw2dcm(csm_result(2));
		ddicp(0, 3) = csm_result(0);
		ddicp(1, 3) = csm_result(1);

		cout << "ddicp[0] = " << endl << ddicp << endl << endl;

		// Subtract the effect of the initial pose. This is how CSM works unfortunately.
		ddicp = csm_init_pose_se3.inverse() * ddicp;

		cout << "ddicp[1] = " << endl << ddicp << endl << endl;

		// * trans. from 1st robot frame to 0th robot frame
		ddicp = sensor_relative_pose * ddicp * sensor_relative_pose_inv;

		cout << "sensor_relative_pose = " << sensor_relative_pose << endl;
		cout << "sensor_relative_pose_inv = " << sensor_relative_pose_inv << endl;

		cout << "ddicp[2] = " << endl << ddicp << endl << endl;

		//ddicp(0, 2) = 0;
		//ddicp(1, 2) = 0;
		//ddicp(2, 0) = 0;
		//ddicp(2, 1) = 0;

		dicp = dicp * ddicp;

		cout << "dicp = " << endl << dicp << endl << endl;

		t_update[idx2]   = ddicp.topRightCorner<3, 1>().norm();
		rot_update[idx2] = utils::trans::dcm2rpy(dicp.topLeftCorner<3, 3>()).cwiseAbs().maxCoeff();

		if(t_update[0] + t_update[1] < _xyz_tol && rot_update[0] + rot_update[1] < _rot_tol)
			break;
	}

	cout << ">>> dicp = " << endl << dicp << endl;

	_pose = _pose * dicp;

	return dicp;
}
