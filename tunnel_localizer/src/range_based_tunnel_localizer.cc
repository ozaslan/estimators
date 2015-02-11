/*
	See the header file for detailed explanations
	of the below functions.
*/

#include "range_based_tunnel_localizer.hh"

RangeBasedTunnelLocalizer::RangeBasedTunnelLocalizer(){
	_pc	        = pcl::PointCloud<pcl::PointXYZ>().makeShared();
	_pc_aligned = pcl::PointCloud<pcl::PointXYZ>().makeShared();
}

bool RangeBasedTunnelLocalizer::reset(){
	_pc->points.clear();
	_num_laser_pushes = 0;
	_num_rgbd_pushes  = 0;
	_pose *= 0;
	_fim *= 0;
	_fitness_score = 0;
	return true;
}

bool RangeBasedTunnelLocalizer::set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &map){
	_icp.setInputTarget(map);
	_octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.05);
	_octree->setInputCloud(map);
	_octree->addPointsFromInputCloud();
	return true;
}

bool RangeBasedTunnelLocalizer::push_laser_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::LaserScan &data, const vector<char> &mask, char cluster_id){
	// ### This still does not incorporate the color information
	assert(mask.size() == 0 || data.ranges.size() == mask.size());

	// Reserve required space to prevent repetitive memory allocation
	_pc->points.reserve(_pc->points.size() + data.ranges.size());

	//cout << "-- L1" << endl;
	//cout << _pc->points.size() << endl;
	//cout << "cluster_id = " << (int)cluster_id << endl;

	Eigen::Vector4d pt;
	double th = data.angle_min;
	for(int i = 0 ; i < (int)data.ranges.size() ; i++, th += data.angle_increment){
		if(mask.size() != 0 && mask[i] != cluster_id)
			continue;
		utils::polar2euclidean(data.ranges[i], th, pt(0), pt(1));
		pt(2) = pt(3) = 0;
		pt = rel_pose * pt;
		_pc->points.push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
	}

	//cout << "-- L2" << endl;
	//cout << "# of pts in the cloud : " << _pc->points.size() << endl;

	// Accumulate information due to the laser scanner onto the _fim matrix.
	// Each additional information is in the body frame. In the 'get_covariance(...)'
	// function (points have to be registered) this is projected onto the world 
	// frame.
	Eigen::Matrix3d fim_xyz = Eigen::Matrix3d::Zero();			// Fisher information for x, y, z coords.
	Eigen::Matrix3d dcm		= rel_pose.topLeftCorner<3, 3>();	// Rotation matrix
	Eigen::Matrix3d fim_xyp;									// Fisher information for x, y, yaw
	double fi_p = 0;											// Fisher information for yaw only (neglecting correlative information)
	
	//cout << "-- L3" << endl;
	utils::get_fim(data, mask, fim_xyp, cluster_id);

	//cout << fim_xyp << endl;
	//cout << fim_xyp.inverse() << endl;
	
	//cout << "-- L4" << endl;
	fim_xyz.topLeftCorner<2, 2>() = fim_xyp.topLeftCorner<2, 2>();
	
	//cout << "-- L5" << endl;
	fim_xyz = dcm.transpose() * fim_xyz * dcm;
	
	//cout << "-- L6" << endl;

	fi_p = fim_xyp(2, 2) * dcm(2, 2);

	_fim.block<3, 3>(0, 0) += fim_xyz;
	_fim(3, 3) += fi_p;


	//cout << "-- L7" << endl;

	_num_laser_pushes++;
	return true;
}

bool RangeBasedTunnelLocalizer::push_rgbd_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::PointCloud2 &data, const vector<char> &mask, char m){
	// ### This still does not incorporate the color information
	assert(mask.size() == 0 || data.data.size() == mask.size());

	// Reserve required space to prevent repetitive memory allocation
	_pc->points.reserve(_pc->points.size() + data.data.size());

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

bool RangeBasedTunnelLocalizer::estimate_pose(const Eigen::Matrix4d &init_pose){
	
	// ### This part might need some optimization.
	// If this class is used with PF, then according to previous
	// transformations, new transformations can be concatenated 
	// easily and efficiently without copying the whole data
	//cout << "-- R0" << endl;
	//static pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed = pcl::PointCloud<pcl::PointXYZ>().makeShared();
	_pc_aligned = pcl::PointCloud<pcl::PointXYZ>().makeShared();
	//pcl::copyPointCloud(*_pc, *_pc_aligned);
	pcl::transformPointCloud(*_pc, *_pc_aligned, init_pose);
	/*
	_icp.setInputSource(pc_transformed);
	_icp.align(*_pc_aligned);
	_pose = _icp.getFinalTransformation().cast<double>();
	_fitness_score = _icp.getFitnessScore();
	return _icp.hasConverged();
	*/

	// For the time being I start using my own implemetation for updating only
	// y, z and psi since PCL::ICP does not allow fixing DOFs. Usually IMU's
	// roll, pitch estimate is acceptably reliable, however ICP cannot constrain
	// updates along there axis.
	
	// (1) - For each ray, find the point of intersection. Use ray-casting of PCL
	// (2) - Prepare the matrix A(2*i,:) = [p_y, p_z, 1, 0] and A(2*i+1,:) = [p_z, -p_y, 0, 1].
	// and the vector b(:) = [r_y, r_z] where p's are the collective sensor data 
	// points and r's are the results of ray casting.
	// (3) - Solve for A*x = b where x = [cos(th) sin(th), dy, dz]'.
	// (4) - Repeat until b's are small enough.
	// (*) - Use weighing to eliminate outliers.
	int num_pts = _pc_aligned->points.size();
	int num_valid_pts = 0;
	double dTz = 0;
	Eigen::MatrixXd A(num_pts, 3);
	Eigen::VectorXd b(num_pts);
	Eigen::Vector3d x;
	Eigen::Vector3f origin, r;
	Eigen::Vector3d rpy;
	Eigen::Matrix4d curr_pose = init_pose;
	origin(0) = init_pose(0, 3);
	origin(1) = init_pose(1, 3);
	origin(2) = curr_pose(2, 3);	

	_matched_map_pts.resize(num_pts);
	int max_iter = 5;
	for(int iter = 0 ; iter < max_iter ; iter++){
		num_valid_pts = 0;
		for(int i = 0 ; i < num_pts ; i++){
			//cout << "i" << i << endl;
			r(0) = _pc_aligned->points[i].x - origin(0);
			r(1) = _pc_aligned->points[i].y - origin(1);
			r(2) = _pc_aligned->points[i].z - origin(2);
			A(i, 0) =  r(1);
			A(i, 1) = -r(0);
			A(i, 2) =  1;
			//cout << origin << endl;
			//cout << direction << endl;
			pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::AlignedPointTVector voxel_centers;
				_octree->getIntersectedVoxelCenters(origin, r,	voxel_centers , 0);
			//cout << "# of intr. voxels : " << voxel_center_centers.size() << endl;
			if(voxel_centers.size() == 0){
				A.block<1, 3>(i, 0) *= 0;
				b(i)     = 0;
				_matched_map_pts[i].x = std::numeric_limits<float>::infinity();
				_matched_map_pts[i].y = std::numeric_limits<float>::infinity();
				_matched_map_pts[i].z = std::numeric_limits<float>::infinity();
			} else {
				int num_inter = voxel_centers.size();
				pcl::PointXYZ &inter = voxel_centers[num_inter - 1];
				//cout << "# of inter. voxels : " << num_inter << endl;
				
				_matched_map_pts[i] = inter;
				b(i) = inter.y - origin(1);
				double ddTz = inter.z - _pc_aligned->points[i].z;
				dTz += ddTz * exp(-r(2) / r.norm());
				num_valid_pts++;

				Eigen::Vector3d w_vec;
				w_vec(0) = _pc_aligned->points[i].x - inter.x;
				w_vec(1) = _pc_aligned->points[i].y - inter.y;
				w_vec(2) = _pc_aligned->points[i].z - inter.z;
				double weight = exp(-pow(w_vec.squaredNorm(), 1.5));
				b(i) *= weight;
				A.block<1, 3>(i, 0) *= weight;


				//double wy = voxel_centers[0].y - _pc_aligned->points[i].y;
				//double wz = voxel_centers[0].z - _pc_aligned->points[i].z;
				//double w  = 1/sqrt(wz * wz + wy * wy + 0.00001);
				//A.block<2, 4>(2*i, 0) *= w;
			}
		}
		//cout << A << endl;
		//cout << b << endl;
		cout << "iter = " << iter << endl;
		cout << "x : " << endl;
		x = (A.transpose() * A).inverse() * A.transpose() * b;
		cout << x << endl;

		dTz /= num_valid_pts;
		x(0) = fabs(x(0)) > 1 ? x(0) / fabs(x(0)) : x(0);
		//double dyaw = -acos(x(0));
		double dyaw = -1 * atan2(x(1), x(0));
		cout << "dyaw = " << dyaw << endl;

		// Update the position
		origin(1) += x(2);
		origin(2) += dTz;

		// Convert the orientation to 'rpy', update yaw and 
		// convert back to dcm.
		Eigen::Matrix3d dcm = curr_pose.topLeftCorner(3, 3);
		rpy = utils::dcm2rpy(dcm);
		rpy(2) += dyaw;
		dcm = utils::rpy2dcm(rpy);

		// Update the global pose matrix.
		curr_pose.topLeftCorner(3, 3) = dcm;
		curr_pose(1, 3) = origin(1);
		curr_pose(2, 3) = origin(2);
	
		cout << "curr_pose : " << curr_pose << endl;
	
		// Transform points.
		pcl::transformPointCloud(*_pc, *_pc_aligned, curr_pose);
	}

	_pose = curr_pose;

	return true;
}

bool RangeBasedTunnelLocalizer::get_pose(Eigen::Matrix3d &dcm , Eigen::Vector3d &pos){
	dcm = _pose.topLeftCorner<3, 3>().cast<double>();
	pos = _pose.topRightCorner<3, 1>().cast<double>();
	return true;
}

bool RangeBasedTunnelLocalizer::get_registered_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc){	
	pc = _pc_aligned;
	return true;
}

bool RangeBasedTunnelLocalizer::get_covariance(Eigen::Matrix6d &cov){
	// ### I have to find a way to project uncertainties in orientation
	// to other frame sets.
	// ### I might have to fix some elements before inverting
	for(int i = 0 ; i < 6 ; i++){
		if(fabs(_fim(i, i)) < 0.01)
			_fim(i, i) = 0.01;
	}
	cov = _fim.inverse();
	return true;
}

double RangeBasedTunnelLocalizer::get_fitness_score(){
	return _fitness_score;
}

bool RangeBasedTunnelLocalizer::get_sensor_map_correspondences(vector<pcl::PointXYZ> &sensor_pts, vector<pcl::PointXYZ> &map_pts){
	map_pts = _matched_map_pts;
	sensor_pts.resize(_pc_aligned->points.size());
	for(int i = 0 ; i < (int)_pc_aligned->points.size() ; i++)
		sensor_pts[i] = _pc_aligned->points[i];
	return true;
}
