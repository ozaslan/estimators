/*
   See the header file for detailed explanations
   of the below functions.
 */

#include "range_based_roof_localizer.hh"

RangeBasedRoofLocalizer::RangeBasedRoofLocalizer(int max_iter, double xyz_tol, double rot_tol){
  _cloud	       = pcl::PointCloud<pcl::PointXYZ>().makeShared();
  _cloud_aligned = pcl::PointCloud<pcl::PointXYZ>().makeShared();
  _max_iter = max_iter;
  _xyz_tol = xyz_tol;
  _rot_tol = rot_tol;
}

bool RangeBasedRoofLocalizer::_reset(){
  _cloud->points.clear();
  _cloud_aligned->points.clear();
  _num_rgbd_pushes  = 0;
  _fim = Eigen::Matrix6d::Zero();

  _laser_procs.clear();

  return true;
}

bool RangeBasedRoofLocalizer::push_laser_data(const LaserProc &laser_proc, bool clean_start){
  if(clean_start == true)
    _reset();

  _laser_procs.push_back(&laser_proc);

  // Accumulate information due to the laser scanner onto the _fim matrix.
  // Each additional information is in the body frame. In the 'get_covariance(...)'
  // function (if points have to been registered) this is projected onto the world 
  // frame.
  Eigen::Matrix3d fim_xyz = Eigen::Matrix3d::Zero();			// Fisher information for x, y, z coords.
  Eigen::Matrix3d dcm		  = laser_proc.get_calib_params().relative_pose.topLeftCorner<3, 3>();	// Rotation matrix
  Eigen::Matrix3d fim_xyp;									// Fisher information for x, y, yaw
  double fi_p = 0;											// Fisher information for yaw only (neglecting correlative information)

  fim_xyp = laser_proc.get_fim();

  fim_xyz.topLeftCorner<2, 2>() = fim_xyp.topLeftCorner<2, 2>();

  fim_xyz = dcm.transpose() * fim_xyz * dcm;

  fi_p = fim_xyp(2, 2) * dcm(2, 2);

  _fim.block<3, 3>(0, 0) += fim_xyz;
  _fim(3, 3) += fi_p;

  return true;
}


/* DEPRECATED */
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

  int num_laser_pushes = _laser_procs.size();
  // The below set of vectors are populated by ray casting
  // each LaserProc data onto the map. Hence the length of
  // these vectors are of the same size as the _laser_procs
  // vector.
  vector<vector<int> >      masks(num_laser_pushes);
  vector<vector<double> >	  ranges(num_laser_pushes);
  vector<vector<double> >	  ths(num_laser_pushes);

  vector<Eigen::Matrix4d> sensor_relative_poses(num_laser_pushes);
  vector<Eigen::Matrix4d> sensor_relative_poses_inv(num_laser_pushes);

  for(int i = 0 ; i < num_laser_pushes ; i++){
    masks[i] = _laser_procs[i]->get_mask();
    for(int j = 0 ; j < (int)masks[i].size() ; j++)
      masks[i][j] = masks[i][j] <= 0 ? 0 : 1;
    ranges[i].resize(masks[i].size());
    ths[i].resize(masks[i].size());
    sensor_relative_poses[i] = _laser_procs[i]->get_calib_params().relative_pose;
    sensor_relative_poses_inv[i] =  sensor_relative_poses[i].inverse();
  }

  /* up until here, I did only initialization ----------------------------------- */
  /* ---------------------------------------------------------------------------- */

  // * robot pose in world frame
  Eigen::Matrix4d	pose = init_pose;
  // * sensor pose (and its inverse) in world frame
  Eigen::Matrix4d sensor_pose, sensor_pose_inv;
  // * sensor pose in SE(3) with zero translation for SE(3) transformations
  Eigen::Matrix4d sensor_relative_pose_no_trans, 
                  sensor_relative_pose_no_trans_inv;
  sensor_relative_pose_no_trans     = Eigen::Matrix4d::Identity();
  sensor_relative_pose_no_trans_inv = Eigen::Matrix4d::Identity();
  // * rotation component of the sensor pose
  Eigen::Matrix3d sensor_pose_dcm, sensor_pose_inv_dcm;
  // * translation component of the sensor pose
  Eigen::Vector3d sensor_pose_t;
  // * pose update at every iteration
  Eigen::Matrix4d dpose;

  // * Temporary rotation matrix etc. used for 3D ray projection.
  Eigen::Vector3d csm_init_pose, temp;
  Eigen::Vector5d csm_result;

  csm_init_pose(0) = 0; // x
  csm_init_pose(1) = 0; // y
  csm_init_pose(2) = 0; // apply the rotation in the below loop instead

  // * translational and rotational updates at each iteration
  vector<double>   t_update(num_laser_pushes);
  vector<double> rot_update(num_laser_pushes);

  // * CSM parameters (hardcoded for now)
  bool    recover_from_error = true;
  double  max_angular_correction_deg = 20;
  double  max_linear_correction = .5;
  int     max_iterations = 1000;
  double  epsilon_xy = 0.001;
  double  epsilon_theta = 0.001;
  double  max_correspondence_dist =  2;
  double  sigma = 0.01;
  bool    use_corr_tricks = false;
  bool    restart = true;
  double  restart_threshold_mean_error = 0.05;
  double  restart_dt = 0.05;
  double  restart_dtheta = 5 * 0.0261799;


  static int index = 1;

  for(int iter = 0 ; iter < _max_iter * num_laser_pushes ; iter++, index++){
    cout << "% ------------------------------------------------------%" << endl;
    cout << "iter = " << iter << ";" << endl;

    // * index of the current contained set
    int idx = iter % num_laser_pushes;

    // * polar -> 3D Euclidean converted laser rays
    const vector<Eigen::Vector3d> &_3d_rays	 = _laser_procs[idx]->get_3d_points();

    // * Current sensor pose and its inverse
    sensor_pose         = pose * sensor_relative_poses[idx];
    sensor_pose_t       = sensor_pose.topRightCorner<3, 1>();
    sensor_pose_dcm     = sensor_pose.topLeftCorner<3, 3>();
    sensor_pose_inv     = sensor_pose.inverse();
    sensor_pose_inv_dcm = sensor_pose_inv.topLeftCorner<3, 3>();
    sensor_relative_pose_no_trans.topLeftCorner<3, 3>()     = sensor_relative_poses    [idx].topLeftCorner<3, 3>();
    sensor_relative_pose_no_trans_inv.topLeftCorner<3, 3>() = sensor_relative_poses_inv[idx].topLeftCorner<3, 3>();
 
    /*
    cout << "pose = " << endl << pose << endl;
    cout << "sensor_pose = " << endl << sensor_pose << endl;
    cout << "sensor_pose_t = " << endl << sensor_pose_t << endl;
    cout << "sensor_pose_dcm = " << endl << sensor_pose_dcm << endl;
    cout << "sensor_relative_pose_no_trans = " << endl << sensor_relative_pose_no_trans << endl;
    cout << "sensor_relative_pose_no_trans_inv = " << endl << sensor_relative_pose_no_trans_inv << endl;
    */

    // Apply the initial transformation on the lidar data 
    // and re-calculate the angles
    octomap::point3d cast_origin(sensor_pose_t(0), sensor_pose_t(1), sensor_pose_t(2));
    octomap::point3d cast_direction, cast_end;

    //cout << "cast_origin{" << index << "} = [" << cast_origin << "]" << endl;
    //cout << "cast_end{" << index << "} = [" << endl;
    for(int i = 0 ; i < (int)_3d_rays.size() ; i++){
      temp = sensor_pose_dcm * _3d_rays[i];
      cast_direction.x() = temp(0);
      cast_direction.y() = temp(1);
      cast_direction.z() = temp(2);
      bool suc = octomap.castRay(cast_origin, cast_direction, cast_end, true, 19.0 /* max range */);
      //cout << cast_end.x() << ", " << cast_end.y() << ", " << cast_end.z() << ";" << endl;
      if(suc == true){
        temp(0) = cast_end.x();
        temp(1) = cast_end.y();
        temp(2) = cast_end.z();
        temp = sensor_pose_inv_dcm * (temp - sensor_pose_t); 
        temp(2) = 0;
        ranges[idx][i] = temp.norm();
        ths[idx][i] = atan2(temp(1), temp(0));
        masks[idx][i] = 1;
      } else {
        masks[idx][i] = 0;
      }
    }
    //cout << "];" << endl;

    // Find 2D transformations with CSM 
    csm_result = utils::laser::register_scan( ranges[idx], masks[idx], ths[idx],
                _laser_procs[idx]->get_ranges(), _laser_procs[idx]->get_mask(), _laser_procs[idx]->get_thetas(), 
                csm_init_pose, recover_from_error, max_angular_correction_deg, max_linear_correction, 
                max_iterations, epsilon_xy, epsilon_theta, max_correspondence_dist, sigma, use_corr_tricks, restart, restart_threshold_mean_error, 
                restart_dt, restart_dtheta); 

    //csm_result(0) *= -1;
    //csm_result(1) *= -1;
    //csm_result(2) *= -1;
    
    /*
    cout << "csm_result{" << index << "} = [" << csm_result << "];" << endl;

    cout << "m{" << index << "} = [ " << endl;
    for(int i = 0 ; i < (int)ranges[idx].size() ; i++){
      if(masks[idx][i] != 0)
        cout << ranges[idx][i] << ", " << ths[idx][i] << ", " << 0 << endl;
      if(_laser_procs[idx]->get_mask()[i] > 0) 
        cout << _laser_procs[idx]->get_ranges()[i] << ", " << _laser_procs[idx]->get_thetas()[i] << ", " << 1 << endl;
    }
    cout << "];" << endl;
    */

    if(csm_result(3) == 0){
      //cout << "invalid CSM" << endl;
      cout << csm_result(4) << endl;
      continue;
    }

    // * trans from 1st sensor frame to 0th sensor frame
    dpose = Eigen::Matrix4d::Identity();
    dpose.topLeftCorner<3, 3>() = utils::trans::yaw2dcm(csm_result(2));
    dpose(0, 3) = csm_result(0);
    dpose(1, 3) = csm_result(1);
    //dpose = dpose.inverse();

    // cout << "dpose[0] = " << endl << dpose << endl << endl;

    // * trans. from 1st robot frame to 0th robot frame
    dpose = sensor_relative_pose_no_trans * dpose * sensor_relative_pose_no_trans_inv;

    // cout << "dpose[1] = " << endl << dpose << endl << endl;

    pose = pose * dpose;

    // cout << "pose = " << endl << pose << endl << endl;

    cout << "yaw{" << index << "} = " << utils::trans::dcm2rpy(pose.topLeftCorner<3, 3>())(2) / PI * 180 << endl;

    t_update[idx]   = dpose.topRightCorner<3, 1>().norm();
    rot_update[idx] = utils::trans::dcm2rpy(dpose.topLeftCorner<3, 3>()).cwiseAbs().maxCoeff();

    if( iter >= num_laser_pushes &&
        std::accumulate(  t_update.begin(),   t_update.end(), 0) < _xyz_tol && 
        std::accumulate(rot_update.begin(), rot_update.end(), 0) < _rot_tol){
        //cout << "exiting iterations." << endl;
        break;
      }
  }

  _pose = pose;

  return 0;
}

bool RangeBasedRoofLocalizer::get_pose(Eigen::Matrix4d &pose){
  pose = _pose;
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
