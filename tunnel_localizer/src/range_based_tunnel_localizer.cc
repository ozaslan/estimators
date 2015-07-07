/*
   See the header file for detailed explanations
   of the below functions.
 */

#include "range_based_tunnel_localizer.hh"

RangeBasedTunnelLocalizer::RangeBasedTunnelLocalizer(int max_iter, double yz_tol, double yaw_tol){
	_cloud	       = pcl::PointCloud<pcl::PointXYZ>().makeShared();
	_cloud_aligned = pcl::PointCloud<pcl::PointXYZ>().makeShared();
	_max_iters = max_iter;
	_yz_tol = yz_tol;
	_yaw_tol = yaw_tol;
}

bool RangeBasedTunnelLocalizer::_reset(){
	_cloud->points.clear();
	_cloud_aligned->points.clear();
	_num_laser_pushes = 0;
	_num_rgbd_pushes  = 0;
	_fim = Eigen::Matrix6d::Zero();
	return true;
}

bool RangeBasedTunnelLocalizer::set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &map){
	// ### This will cause memory leak!!!
	_octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.05);
	_octree->setInputCloud(map);
	_octree->addPointsFromInputCloud();
	return true;
}

bool RangeBasedTunnelLocalizer::push_laser_data(const LaserProc &laser_proc, bool clean_start){
	const vector<int> &mask = laser_proc.get_mask();
	const vector<Eigen::Vector3d> &_3d_pts = laser_proc.get_3d_points();

	if(clean_start == true)
		_reset();

	// Reserve required space to prevent repetitive memory allocation
	_cloud->points.reserve(_cloud->points.size() + mask.size());

	for(int i = 0 ; i < (int)mask.size() ; i++){
		if(mask[i] <= 0)
			continue;
		_cloud->points.push_back(pcl::PointXYZ(_3d_pts[i](0), _3d_pts[i](1), _3d_pts[i](2)));
	}

	// Accumulate information due to the laser scanner onto the _fim matrix.
	// Each additional information is in the body frame. In the 'get_covariance(...)'
	// function (if points have to been registered) this is projected onto the world 
	// frame.
	Eigen::Matrix3d fim_xyz = Eigen::Matrix3d::Zero();      // Fisher information for x, y, z coords.
	Eigen::Matrix3d dcm     = laser_proc.get_calib_params().relative_pose.topLeftCorner<3, 3>();  // Rotation matrix
	Eigen::Matrix3d fim_xyp;                                // Fisher information for x, y, yaw
	double fi_p = 0;                                        // Fisher information for yaw only (neglecting correlative information)

	fim_xyp = laser_proc.get_fim();

	//cout << "fim_xyp = " << fim_xyp << endl;

	fim_xyz.topLeftCorner<2, 2>() = fim_xyp.topLeftCorner<2, 2>();

	fim_xyz = dcm * fim_xyz * dcm.transpose();

	fi_p = fim_xyp(2, 2) * fabs(dcm(2, 2));

	//cout << "dcm = [" << dcm << "];" << endl;

	_fim.block<3, 3>(0, 0) += fim_xyz;
	_fim(5, 5) += fi_p;

	//cout << "_fim = [" << _fim << "];" << endl;

	return true;

}

bool RangeBasedTunnelLocalizer::push_laser_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::LaserScan &data, const vector<char> &mask, char cluster_id, bool clean_start){
	// ### This still does not incorporate the color information
	ASSERT(mask.size() == 0 || data.ranges.size() == mask.size(), "mask and data size should be the same.");
	ASSERT(cluster_id != 0, "Cluster \"0\" is reserved.");

	if(clean_start == true)
		_reset();

	// Reserve required space to prevent repetitive memory allocation
	_cloud->points.reserve(_cloud->points.size() + data.ranges.size());

	Eigen::Vector4d pt;
	double th = data.angle_min;
	for(int i = 0 ; i < (int)data.ranges.size() ; i++, th += data.angle_increment){
		if(mask.size() != 0 && mask[i] != cluster_id)
			continue;
		utils::laser::polar2euclidean(data.ranges[i], th, pt(0), pt(1));
		pt(2) = pt(3) = 0;
		pt = rel_pose * pt;
		_cloud->points.push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
	}

	// Accumulate information due to the laser scanner onto the _fim matrix.
	// Each additional information is in the body frame. In the 'get_covariance(...)'
	// function (if points have to been registered) this is projected onto the world 
	// frame.
	Eigen::Matrix3d fim_xyz = Eigen::Matrix3d::Zero();			// Fisher information for x, y, z coords.
	Eigen::Matrix3d dcm		= rel_pose.topLeftCorner<3, 3>();	// Rotation matrix
	Eigen::Matrix3d fim_xyp;									// Fisher information for x, y, yaw
	double fi_p = 0;											// Fisher information for yaw only (neglecting correlative information)

	utils::laser::get_fim(data, mask, fim_xyp, cluster_id);

	fim_xyz.topLeftCorner<2, 2>() = fim_xyp.topLeftCorner<2, 2>();

	fim_xyz = dcm.transpose() * fim_xyz * dcm;

	fi_p = fim_xyp(2, 2) * dcm(2, 2);

	_fim.block<3, 3>(0, 0) += fim_xyz;
	_fim(5, 5) += fi_p;

	_num_laser_pushes++;

	return true;
}

bool RangeBasedTunnelLocalizer::push_rgbd_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::PointCloud2 &data, const vector<char> &mask, char cluster_id, bool clean_start){
	// ### This still does not incorporate the color information
	ASSERT(mask.size() == 0 || data.data.size() == mask.size(), "mask and data size should be the same.");
	ASSERT(cluster_id != 0, "Cluster \"0\" is reserved.");

	if(clean_start == true)
		_reset();

	// Reserve required space to prevent repetitive memory allocation
	_cloud->points.reserve(_cloud->points.size() + data.data.size());

	Eigen::Vector4d pt;

	ROS_WARN("\"push_rgbd_data(...)\" is not implemented yet!");
	for(int i = 0 ; i < (int)data.data.size() ; i++){
		if(mask.size() != 0 || mask[i] == false)
			continue;
		// ### To be implemented!
	}

	_num_rgbd_pushes++;
	return true;
}

int RangeBasedTunnelLocalizer::estimate_pose(const Eigen::Matrix4d &init_pose){
	// ### This will cause memory leak!!!
	_cloud_aligned = pcl::PointCloud<pcl::PointXYZ>().makeShared();
	// Initialize '_cloud_aligned' by transforming the collective point cloud
	// with the initial pose.
	pcl::transformPointCloud(*_cloud, *_cloud_aligned, init_pose);

	// Steps : 
	// (1) - For each ray, find the point of intersection. Use ray-casting of PCL-Octree
	// (2) - Prepare the matrix A and vector b s.t.  A*x = b
	// (3) - Solve for A*x = b where x = [cos(th) sin(th), dy, dz]'.
	// (4) - Repeat for _max_iter's or tolerance conditions are met
	// (*) - Use weighing to eliminate outliers.
	int num_pts = _cloud_aligned->points.size();
	int num_valid_pts = 0;	// # of data points intersecting with the map.
	double dTz = 0;			// update along the z-coordinate.

	Eigen::MatrixXd A(num_pts, 3);
	Eigen::VectorXd b(num_pts);
	Eigen::Vector3d x, rpy;	// solution to Ax=b, 
	// roll-pitch-yaw
	Eigen::Vector3f pos;	// initial position of the robot.
	// This is given as argument to ray-caster.
	Eigen::Vector3f ray;
	Eigen::Vector3d res_vec;

	Eigen::Matrix4d curr_pose = init_pose; // 'current' pose after each iteration.

	// container for ray-casting results :
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::AlignedPointTVector voxel_centers;

	pos = curr_pose.topRightCorner<3, 1>().cast<float>();

	_matched_map_pts.resize(num_pts);

	int iter;
	for(iter = 0 ; iter < _max_iters ; iter++){
		//cout << "projection pos = " << pos << endl;
		num_valid_pts = 0;
		_fitness_scores[0] = _fitness_scores[1] = _fitness_scores[2] = 0;
		for(int i = 0 ; i < num_pts ; i++){
			// Get the ray in the body frame ()
			ray(0) = _cloud_aligned->points[i].x - pos(0);
			ray(1) = _cloud_aligned->points[i].y - pos(1);
			ray(2) = _cloud_aligned->points[i].z - pos(2);
			A(i, 0) =  ray(1);
			A(i, 1) = -ray(0);
			A(i, 2) =  1;
			// Fetch only the first intersection point
			_octree->getIntersectedVoxelCenters(pos, ray, voxel_centers , 1);
			// If there is no intersection, nullify the effect by zero'ing the corresponding equation
			if(voxel_centers.size() == 0){
				A.block<1, 3>(i, 0) *= 0;
				b(i)				 = 0;
				_matched_map_pts[i].x = 
					_matched_map_pts[i].y = 
					_matched_map_pts[i].z = std::numeric_limits<float>::infinity();
			} else {
				// Save the matched point
				_matched_map_pts[i] = voxel_centers[0];
				// Use only the y-comp of the residual vector. Because 
				// this is going to contribute to y and yaw updates only.
				b(i) = voxel_centers[0].y - pos(1);
				// Get the residual vector 
				res_vec(0) = _cloud_aligned->points[i].x - voxel_centers[0].x;
				res_vec(1) = _cloud_aligned->points[i].y - voxel_centers[0].y;
				res_vec(2) = _cloud_aligned->points[i].z - voxel_centers[0].z;
				// Update the delta-z-estimate according to the direction of the 
				// corresponding ray's z component. The update factor is weighed using
				// ML outlier elimination.
				dTz += -res_vec(2) * (exp(fabs(ray(2) / ray.norm())) - 1);
				// Calculate a weighing coefficent for ML outlier elimination
				// regarding y-yaw DOFs
				double weight = exp(-pow(res_vec.squaredNorm(), 1.5));
				b(i) *= weight;
				A.block<1, 3>(i, 0) *= weight;

				num_valid_pts++;

				_fitness_scores[0] += res_vec[1] * res_vec[1];
				_fitness_scores[1] += res_vec[2] * res_vec[2];
				_fitness_scores[2] += res_vec[1] * res_vec[1] * ray(0) * ray(0);
			}
		}

		// Solve for the least squares solution.
		x = (A.transpose() * A).inverse() * A.transpose() * b;

		// Get the mean of dTz since it has been accumulated
		dTz /= num_valid_pts;
		_fitness_scores[0] /= num_valid_pts;
		_fitness_scores[1] /= num_valid_pts;
		_fitness_scores[2] /= num_valid_pts;

		// x = [cos(yaw) sin(yaw) dY]
		x(0) = fabs(x(0)) > 1 ? x(0) / fabs(x(0)) : x(0);
		double dyaw = -1.2 * atan2(x(1), x(0));

		// Update the position
		pos(1) += x(2); // y-update 
		pos(2) += dTz;	// z-update

		// Convert the orientation to 'rpy', update yaw and 
		// convert back to dcm.
		Eigen::Matrix3d dcm = curr_pose.topLeftCorner<3, 3>();
		rpy = utils::trans::dcm2rpy(dcm);
		rpy(2) += dyaw;
		dcm = utils::trans::rpy2dcm(rpy);

		// Update the global pose matrix.
		curr_pose.topLeftCorner(3, 3) = dcm;
		curr_pose(1, 3) = pos(1);
		curr_pose(2, 3) = pos(2);

		// Transform points for next iteration.
		pcl::transformPointCloud(*_cloud, *_cloud_aligned, curr_pose);

		if(fabs(x(2)) < _yz_tol && fabs(dTz) < _yz_tol && fabs(dyaw) < _yaw_tol)
			break;
	}

	_pose = curr_pose;

	return iter;
}

bool RangeBasedTunnelLocalizer::get_pose(Eigen::Matrix4d &pose){
	pose = _pose.cast<double>();
	return true;
}

bool RangeBasedTunnelLocalizer::get_registered_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc){	
	pc = _cloud_aligned;
	return true;
}

bool RangeBasedTunnelLocalizer::get_covariance(Eigen::Matrix6d &cov, bool apply_fitness_result){
	// ### I have to find a way to project uncertainties in orientation
	// to other frame sets.
	// ### I might have to fix some elements before inverting
	Eigen::EigenSolver<Eigen::Matrix6d> es;
	es.compute(_fim, true);
	Eigen::MatrixXd D = es.eigenvalues().real().asDiagonal();
	Eigen::MatrixXd V = es.eigenvectors().real();


	bool reconstruct = false;
	for(int i = 0 ; i < 6 ; i++)
		if(D(i, i) < 0.00001){
			D(i, i) = 0.00001;
			reconstruct = true;
		}

	if(reconstruct)
		_fim = (V.transpose() * D * V).real();

	//cout << "FIM = [" << _fim << "];" << endl;

	// ### This assumes uniform covariance for each rays, and superficially
	// handles the uncertainties. I should first run ICP and then
	// find '_fim' and so on. This will require a lot of bookkeeping etc.
	// Thus leaving to a later version :)
	if(apply_fitness_result){
		// ### Is the rotation ypr/rpy ???
		// We assume that 0.001 m^2 variance is perfect fit, or unit information.
		D(0, 0) /= exp(_fitness_scores[0] / 0.001);
		D(1, 1) /= exp(_fitness_scores[1] / 0.001);
		D(2, 2) /= exp(_fitness_scores[2] / 0.001);
	}

	for(int i = 0 ; i < 6 ; i++)
		D(i, i) = 1 / D(i, i) + 0.00001;
	cov = (V * D * V.transpose());

	//cout << "V = [" << V << "];" << endl;
	//cout << "D = [" << D << "];" << endl;
	//cout << "cov = [ " << cov << "];" << endl;

	cov.topLeftCorner<3, 3>	() = _pose.topLeftCorner<3, 3>().transpose() * 
									cov.topLeftCorner<3, 3>() * 
								 _pose.topLeftCorner<3, 3>();

	return true;
}

bool RangeBasedTunnelLocalizer::get_fitness_scores(double &y, double &z, double &yaw){
	y   = _fitness_scores[0];
	z   = _fitness_scores[1];
	yaw = _fitness_scores[2];
	return true;
}

bool RangeBasedTunnelLocalizer::get_correspondences(vector<pcl::PointXYZ> &sensor_pts, vector<pcl::PointXYZ> &map_pts){
	int num_valid_pts = 0;
	for(int i = 0 ; i < (int)_matched_map_pts.size() ; i++)
		if(isfinite(_matched_map_pts[i].y))
			num_valid_pts++;
	map_pts.resize(num_valid_pts);
	sensor_pts.resize(num_valid_pts);
	for(int i = 0, j = 0 ; i < (int)_cloud_aligned->points.size() ; i++){
		if(isfinite(_matched_map_pts[i].y)){
			sensor_pts[j] = _cloud_aligned->points[i];
			map_pts[j] = _matched_map_pts[i];
			j++;
		}
	}
	return true;
}
