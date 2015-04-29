/*
	See the header file for detailed explanations
	of the below functions.
*/

#include "range_based_roof_localizer.hh"

RangeBasedRoofLocalizer::RangeBasedRoofLocalizer(int max_iter, double xyz_tol, double yaw_tol){
	_cloud	       = pcl::PointCloud<pcl::PointXYZ>().makeShared();
	_cloud_aligned = pcl::PointCloud<pcl::PointXYZ>().makeShared();
	_max_iters = max_iter;
	_xyz_tol = xyz_tol;
	_yaw_tol = yaw_tol;
}

bool RangeBasedRoofLocalizer::_reset(){
	_cloud->points.clear();
	_cloud_aligned->points.clear();
	_num_laser_pushes = 0;
	_num_rgbd_pushes  = 0;
	_fim = Eigen::Matrix6d::Zero();
	return true;
}

bool RangeBasedRoofLocalizer::push_laser_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::LaserScan &data, const vector<char> &mask, char cluster_id, bool clean_start){
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
	_fim(3, 3) += fi_p;

	_num_laser_pushes++;

	return true;
}

bool RangeBasedRoofLocalizer::push_rgbd_data(const Eigen::Matrix4d &rel_pose, const sensor_msgs::PointCloud2 &data, const vector<char> &mask, char cluster_id, bool clean_start){
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

int RangeBasedRoofLocalizer::estimate_pose(const Eigen::Matrix4d &init_pose, const octomap::OcTree &octomap){
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

	Eigen::MatrixXd A(2 * num_pts, 4);
	Eigen::VectorXd b(2 * num_pts);
	Eigen::Vector4d x;		// solution to Ax=b, 
	Eigen::Vector3d rpy;	// roll-pitch-yaw
	Eigen::Vector3f pos;	// initial position of the robot.
							// This is given as argument to ray-caster.
	//Eigen::Vector3f ray;
	Eigen::Vector3d res_vec;

	octomap::point3d origin, ray, end;

	Eigen::Matrix4d curr_pose = init_pose; // 'current' pose after each iteration.

	//pos = curr_pose.topRightCorner<3, 1>().cast<float>();
	pos.x() = curr_pose(0, 3);
	pos.y() = curr_pose(1, 3);
	pos.z() = curr_pose(2, 3);

	_matched_map_pts.resize(num_pts);

	//cout << "initial pose in the localizer : " << endl;
	//cout << "curr_pose = " << curr_pose << endl;

	int iter;
	for(iter = 0 ; iter < _max_iters ; iter++){
		Eigen::Vector3f prev_point(9999, 9999, 9999);
		cout << "----------------------------" << endl;
		cout << "iter = " << iter << endl;
		//cout << "pose = " << pos  << endl;
		num_valid_pts = 0;
		_fitness_scores[0] = _fitness_scores[1] = _fitness_scores[2] = 0;

		origin.x() = pos(0);
		origin.y() = pos(1);
		origin.z() = pos(2);

		static vector<octomap::point3d> contour;
		static vector<bool> ray_cast_suc;
		contour.resize(num_pts);
		ray_cast_suc.resize(num_pts);
		for(int i = 0 ; i < num_pts ; i++){
			// Get the ray in the body frame ()
			ray.x() = _cloud_aligned->points[i].x - pos(0);
			ray.y() = _cloud_aligned->points[i].y - pos(1);
			ray.z() = _cloud_aligned->points[i].z - pos(2);

			// Fetch only the first intersection point
			//cout << "origin = " << origin << endl;
			//cout << "ray    = " << ray << endl;
			bool suc = octomap.castRay(origin, ray, end, true, 19.0);
			ray_cast_suc[i] = suc;
			if(suc == true)
				contour[i] = end;
			else 
				contour[i].x() = 9999999;
		}

		for(int i = 0 ; i < num_pts ; i++){		
			// If there was no intersection, nullify the effect by zero'ing the corresponding equation
			if(ray_cast_suc[i] == false){
				A.block<2, 4>(2 * i, 0).setZero();
				b(2 * i) = b(2 * i + 1) = 0;
				_matched_map_pts[i].x = 
					_matched_map_pts[i].y = 
					_matched_map_pts[i].z = std::numeric_limits<float>::infinity();
			} else {
				octomap::point3d ray_tip;
				ray_tip.x() = _cloud_aligned->points[i].x;
				ray_tip.y() = _cloud_aligned->points[i].y;
				ray_tip.z() = _cloud_aligned->points[i].z;


				double min_dist = 9999;
				int    min_dist_idx = 0;
				for(int j = 0 ; j < num_pts ; j++){
					double curr_dist = (ray_tip - contour[j]).norm();
					if(curr_dist < min_dist){
						min_dist = curr_dist;
						min_dist_idx = j;
					}
				}

				end = contour[min_dist_idx];

				/* 
					1 - Get the previous and next points.
					2 - Compare attack angle to the corresponding lines.
					3 - Pick the line with the largest dot product value.
					4 - If the dot product is less than 0, the closest point
						is the end-point
					5 - If 4 does not hold dot product the distance to the
						end-point with the normal vector. This becomes the
						point-to-line distance.
				*/	
				
				octomap::point3d prev_pt, next_pt;
				if(min_dist_idx - 1 < 0){
					next_pt = contour[min_dist_idx + 1];
					prev_pt = next_pt;
				} else if(min_dist_idx + 1 >= (int)contour.size()){
					prev_pt = contour[min_dist_idx - 1];
					next_pt = prev_pt;
				} else {
					prev_pt = contour[min_dist_idx - 1];
					next_pt = contour[min_dist_idx + 1];
				}

				double dot1, dot2;
				octomap::point3d ray0, ray1, ray2;
				ray0 = end - ray_tip;
				ray1 = next_pt - end;
				ray2 = prev_pt - end;
				dot1 = ray0.dot(ray1) / ray0.norm() / ray1.norm();
				dot2 = ray0.dot(ray2) / ray0.norm() / ray2.norm();


				if(dot1 < 0 && dot2 < 0){
					// Closest point on the line the 
					// 'end' end-point.	
				} else {
					if(dot1 > dot2){
						end = end + ray0; - ray1 * (ray0.norm() * dot1);
					} else {
						//end = end + ray0 - (ray0.norm() * dot2) * ray2;
					}
				}

				/* */


				//cout << "% match #" << i << endl;
				//cout << ray_tip << endl;
				//cout << end << endl;		

				// Get the ray in the body frame ()
				ray.x() = _cloud_aligned->points[i].x - pos(0);
				ray.y() = _cloud_aligned->points[i].y - pos(1);
				ray.z() = _cloud_aligned->points[i].z - pos(2);
				A(2*i, 0) =  ray.x();
				A(2*i, 1) = -ray.y();
				A(2*i, 2) =  1;
				A(2*i, 3) =  0;
				A(2*i + 1, 0) = ray.y();
				A(2*i + 1, 1) = ray.x();
				A(2*i + 1, 2) = 0;
				A(2*i + 1, 3) = 1;

				// Save the matched point
				_matched_map_pts[i].x = end.x();
				_matched_map_pts[i].y = end.y();
				_matched_map_pts[i].z = end.z();
				// Use only the y-comp of the residual vector. Because 
				// this is going to contribute to y and yaw updates only.
				b(2*i)     = end.x() - pos(0);
				b(2*i + 1) = end.y() - pos(1);
				// Get the residual vector 
				res_vec(0) = _cloud_aligned->points[i].x - end.x();
				res_vec(1) = _cloud_aligned->points[i].y - end.y();
				res_vec(2) = _cloud_aligned->points[i].z - end.z();

				/*
				if(res_vec.norm() > 2.00){
					A.block<2, 4>(2 * i, 0).setZero();
					b(2 * i) = b(2 * i + 1) = 0;
					_matched_map_pts[i].x = 
						_matched_map_pts[i].y = 
						_matched_map_pts[i].z = std::numeric_limits<float>::infinity();
					continue;
				}
				*/

				// Update the delta-z-estimate according to the direction of the 
				// corresponding ray's z component. The update factor is weighed using
				// ML outlier elimination.
				dTz += -res_vec(2) * (exp(pow(fabs(ray(2) / ray.norm()), 1.0) - 1));
				// Calculate a weighing coefficent for ML outlier elimination
				// regarding y-yaw DOFs
				
				double dir_weight = pow(exp(sqrt(ray(0) * ray(0) + ray(1) * ray(1)) / ray.norm()), 2) - 1;
				double weight = exp(-pow(res_vec.squaredNorm(), 2.5));
				weight = res_vec.norm() >= 2.50 ? exp(-2.50) : weight;
				//weight = 1; 
				//double weight = res_vec.norm() > 0.5 ? 0 : 1;
				b(2 * i) *= weight * dir_weight;
				b(2 * i + 1) *= weight * dir_weight;
				A.block<2, 4>(2 * i, 0) *= weight * dir_weight;
				
				//double weight_x = exp(-fabs(res_vec(0) / res_vec.norm()));
				//double weight_y = exp(-fabs(res_vec(1) / res_vec.norm()));
				//b(2 * i) *= weight_x;
				//b(2 * i + 1) *= weight_y;
				//A.block<1, 4>(2 * i, 0) *= weight_x;
				//A.block<1, 4>(2 * i + 1, 0) *= weight_y;
				
				num_valid_pts++;

				_fitness_scores[0] += res_vec[1] * res_vec[1];
				_fitness_scores[1] += res_vec[2] * res_vec[2];
				_fitness_scores[2] += res_vec[1] * res_vec[1] * ray(0) * ray(0);
			}	
		}

		/*
		for(int i = 0 ; i < num_pts ; i++){
			Eigen::Vector3f curr_point(_cloud_aligned->points[i].x,
									   _cloud_aligned->points[i].y,
									   _cloud_aligned->points[i].z);
			if((curr_point - prev_point).norm() <= 0.05){
				A.block<2, 4>(2 * i, 0).setZero();
				b(2 * i) = b(2 * i + 1) = 0;
				_matched_map_pts[i].x = 
					_matched_map_pts[i].y = 
					_matched_map_pts[i].z = std::numeric_limits<float>::infinity();
				continue; //!!!
			} else
				prev_point = curr_point;


			// Get the ray in the body frame ()
			ray.x() = _cloud_aligned->points[i].x - pos(0);
			ray.y() = _cloud_aligned->points[i].y - pos(1);
			ray.z() = _cloud_aligned->points[i].z - pos(2);
			A(2*i, 0) =  ray.x();
			A(2*i, 1) = -ray.y();
			A(2*i, 2) =  1;
			A(2*i, 3) =  0;
			A(2*i + 1, 0) = ray.y();
			A(2*i + 1, 1) = ray.x();
			A(2*i + 1, 2) = 0;
			A(2*i + 1, 3) = 1;

			origin.x() = pos(0);
			origin.y() = pos(1);
			origin.z() = pos(2);

			//cout << "ray = " << ray << endl;
			// Fetch only the first intersection point
			bool suc = octomap.castRay(origin, ray, end, true, 19.0);
		
			//cout << "BBX Max = " << octomap.getBBXMax() << endl;
			cout << "origin = " << origin << endl;
			//cout << "ray    = " << ray    << endl;
			//cout << "end    = " << end    << endl;
			// If there is no intersection, nullify the effect by zero'ing the corresponding equation
			if(suc == false){
				A.block<2, 4>(2 * i, 0).setZero();
				b(2 * i) = b(2 * i + 1) = 0;
				_matched_map_pts[i].x = 
					_matched_map_pts[i].y = 
					_matched_map_pts[i].z = std::numeric_limits<float>::infinity();
			} else {
				// Save the matched point
				_matched_map_pts[i].x = end.x();
				_matched_map_pts[i].y = end.y();
				_matched_map_pts[i].z = end.z();
				// Use only the y-comp of the residual vector. Because 
				// this is going to contribute to y and yaw updates only.
				b(2*i)     = end.x() - pos(0);
				b(2*i + 1) = end.y() - pos(1);
				// Get the residual vector 
				res_vec(0) = _cloud_aligned->points[i].x - end.x();
				res_vec(1) = _cloud_aligned->points[i].y - end.y();
				res_vec(2) = _cloud_aligned->points[i].z - end.z();

				if(res_vec.norm() > 2.00){
					A.block<2, 4>(2 * i, 0).setZero();
					b(2 * i) = b(2 * i + 1) = 0;
					_matched_map_pts[i].x = 
						_matched_map_pts[i].y = 
						_matched_map_pts[i].z = std::numeric_limits<float>::infinity();
					continue;
				}

				// Update the delta-z-estimate according to the direction of the 
				// corresponding ray's z component. The update factor is weighed using
				// ML outlier elimination.
				dTz += -res_vec(2) * (exp(pow(fabs(ray(2) / ray.norm()), 1.0) - 1));
				// Calculate a weighing coefficent for ML outlier elimination
				// regarding y-yaw DOFs
				
				double dir_weight = pow(exp(sqrt(ray(0) * ray(0) + ray(1) * ray(1)) / ray.norm()), 2) - 1;
				double weight = exp(-pow(res_vec.squaredNorm(), 0.5));
				weight = res_vec.norm() >= 2.50 ? exp(-2.50) : weight;
				//weight = 1; 
				//double weight = res_vec.norm() > 0.5 ? 0 : 1;
				b(2 * i) *= weight * dir_weight;
				b(2 * i + 1) *= weight * dir_weight;
				A.block<2, 4>(2 * i, 0) *= weight * dir_weight;
				
				//double weight_x = exp(-fabs(res_vec(0) / res_vec.norm()));
				//double weight_y = exp(-fabs(res_vec(1) / res_vec.norm()));
				//b(2 * i) *= weight_x;
				//b(2 * i + 1) *= weight_y;
				//A.block<1, 4>(2 * i, 0) *= weight_x;
				//A.block<1, 4>(2 * i + 1, 0) *= weight_y;
				
				num_valid_pts++;

				_fitness_scores[0] += res_vec[1] * res_vec[1];
				_fitness_scores[1] += res_vec[2] * res_vec[2];
				_fitness_scores[2] += res_vec[1] * res_vec[1] * ray(0) * ray(0);
			}
		}
		*/

		//cout << "A = " << A << endl;
		//cout << "b = " << b << endl;

		// Solve for the least squares solution.
		x = (A.transpose() * A).inverse() * A.transpose() * b;

		//cout << "x = " << x << endl;

		// Get the mean of dTz since it has been accumulated
		dTz /= num_valid_pts;
		_fitness_scores[0] /= num_valid_pts;
		_fitness_scores[1] /= num_valid_pts;
		_fitness_scores[2] /= num_valid_pts;

		// x = [cos(yaw) sin(yaw) dY]
		x(0) = fabs(x(0)) > 1 ? x(0) / fabs(x(0)) : x(0);
		double dyaw = 1.0 * atan2(x(1), x(0));

		// Update the position
		pos(0) += x(2); // x-update 
		pos(1) += x(3); // y-update 
		pos(2) += dTz;	// z-update

		// Convert the orientation to 'rpy', update yaw and 
		// convert back to dcm.
		Eigen::Matrix3d dcm = curr_pose.topLeftCorner<3, 3>();
		rpy = utils::trans::dcm2rpy(dcm);
		rpy(2) += dyaw;
		//cout << "dyaw = " << dyaw / 3.14 * 180<< endl;
		//cout << "rpy = " << rpy << endl;
		dcm = utils::trans::rpy2dcm(rpy);

		// Update the global pose matrix.
		curr_pose.topLeftCorner<3, 3>() = dcm;
		//pos *= 1.5;
		curr_pose(0, 3) = pos(0);
		curr_pose(1, 3) = pos(1);
		curr_pose(2, 3) = pos(2);
	
		// Transform points for next iteration.
		pcl::transformPointCloud(*_cloud, *_cloud_aligned, curr_pose);

		//break;
		if(fabs(x(2)) < _xyz_tol && fabs(x(3)) < _xyz_tol && fabs(dTz) < _xyz_tol && fabs(dyaw) < _yaw_tol)
			break;
	}

	_pose = curr_pose;

	//cout << "estimated pose in the localizer : " << endl;
	//cout << "_pose = "  << _pose << endl;

	return iter;
}

bool RangeBasedRoofLocalizer::get_pose(Eigen::Matrix4d &pose){
    pose = _pose.cast<double>();
	return true;
}

bool RangeBasedRoofLocalizer::get_registered_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc){	
	pc = _cloud_aligned;
	return true;
}

bool RangeBasedRoofLocalizer::get_covariance(Eigen::Matrix6d &cov, bool apply_fitness_result){
	// ### I have to find a way to project uncertainties in orientation
	// to other frame sets.
	// ### I might have to fix some elements before inverting
	for(int i = 0 ; i < 6 ; i++){
		if(fabs(_fim(i, i)) < 0.01)
			_fim(i, i) = 0.01;
	}

	// ### This assumes uniform covariance for each rays, and superficially
	// handles the uncertainties. I should first run ICP and then
	// find '_fim' and so on. This will require a lot of bookkeeping etc.
	// Thus leaving to a later version :)
	if(apply_fitness_result){
		Eigen::EigenSolver<Eigen::Matrix6d> es;
		es.compute(_fim, true);
		Eigen::MatrixXcd D = es.eigenvalues().asDiagonal();
		Eigen::MatrixXcd V = es.eigenvectors();
		// ### Is the rotation ypr/rpy ???
		// We assume that 0.001 m^2 variance is perfect fit, or unit information.
		D(1, 1) /= exp(_fitness_scores[0] / 0.001);
		D(2, 2) /= exp(_fitness_scores[1] / 0.001);
		D(3, 3) /= exp(_fitness_scores[2] / 0.001);
		cov =  (V.transpose() * D.inverse() * V).real();
	} else
		cov = _fim.inverse();
	return true;
}

bool RangeBasedRoofLocalizer::get_fitness_scores(double &y, double &z, double &yaw){
	y   = _fitness_scores[0];
	z   = _fitness_scores[1];
	yaw = _fitness_scores[2];
	return true;
}

bool RangeBasedRoofLocalizer::get_correspondences(vector<pcl::PointXYZ> &sensor_pts, vector<pcl::PointXYZ> &map_pts){
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
