#include "velodyne_odom.hh"
#include "utils.hh"

/*
   -- Implement overlapping-spheres for loop closure
   -- Integrate ndt fit score into factor sigmas
 */

using namespace std;

int VelodyneOdom::VelodyneOdomParams::print(){
  cout << "VelodyneOdomParams -----------------------" << endl;
  cout << "\t approximate_voxel_leaf_size = [" << approximate_voxel_leaf_size[0] << ", " 
    << approximate_voxel_leaf_size[1] << ", "
    << approximate_voxel_leaf_size[2] << "]" << endl;
  cout << "\t ndt_eps ------ = " << ndt_eps << endl;
  cout << "\t ndt_res ------ = " << ndt_res << endl;
  cout << "\t ndt_max_iter - = " << ndt_max_iter << endl;
  cout << "\t ndt_step_size  = " << ndt_step_size << endl;
  cout << "\t batch_ndt_eps ------ = " << batch_ndt_eps << endl;
  cout << "\t batch_ndt_res ------ = " << batch_ndt_res << endl;
  cout << "\t batch_ndt_max_iter - = " << batch_ndt_max_iter << endl;
  cout << "\t batch_ndt_step_size  = " << batch_ndt_step_size << endl;
  /*
     cout << "\t trans_offset_weight  = " << trans_offset_weight << endl;
     cout << "\t rot_offset_weight -- = [" << rot_offset_weight[0] << ", " 
     << rot_offset_weight[1] << ", "
     << rot_offset_weight[2] << "]" << endl;
   */
  cout << "\t init_keyframe_trans_thres = " << init_keyframe_trans_thres << endl;
  cout << "\t init_keyframe_rot_thres - = " << init_keyframe_rot_thres << endl;
  cout << "------------------------------------------" << endl;
  return 0;
}

void VelodyneOdom::_initialize(){
  _approximate_voxel_filter.setLeafSize (_params.approximate_voxel_leaf_size[0],
      _params.approximate_voxel_leaf_size[1],
      _params.approximate_voxel_leaf_size[2]);

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  _ndt.setTransformationEpsilon (_params.ndt_eps);
  _batch_ndt.setTransformationEpsilon (_params.batch_ndt_eps);
  // Setting maximum step size for More-Thuente line search.
  _ndt.setStepSize (_params.ndt_step_size);
  _batch_ndt.setStepSize (_params.batch_ndt_step_size);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  _ndt.setResolution (_params.ndt_res);
  _batch_ndt.setResolution (_params.batch_ndt_res);
  // Setting max number of registration iterations.
  _ndt.setMaximumIterations (_params.ndt_max_iter);
  _batch_ndt.setMaximumIterations (_params.batch_ndt_max_iter);

  _batch_optimizing = false;

  _map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  _keyframes.clear();
  _keyframe_poses.clear();
  //_keyframes = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  //_keyframe_pose.setIdentity();
  _pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  _filtered_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  _aligned_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  _gtsam_prior_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  _params.print();
}

VelodyneOdom::VelodyneOdom(){
  _initialize();
}

VelodyneOdom::VelodyneOdom(const VelodyneOdomParams &params){
  _params = params;
  _initialize();
}

double VelodyneOdom::_pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double trans_thres, double rot_thres){
  Eigen::Matrix4d dpose = pose1.inverse() * pose2;
  Eigen::Vector3d aaxis = utils::trans::dcm2aaxis(utils::trans::cancel_yaw(Eigen::Matrix3d(dpose.topLeftCorner<3, 3>())));
  return dpose.topRightCorner<3, 1>().norm() / trans_thres + 
    aaxis.norm() / rot_thres;
}

double VelodyneOdom::_pose_distance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2){
  return _pose_distance(pose1, pose2, _params.init_keyframe_trans_thres, _params.init_keyframe_rot_thres);
}

bool VelodyneOdom::_push_new_keyframe_required(const Eigen::Matrix4d &curr_pose, int key_frame_ind){
  ASSERT(key_frame_ind < (int)_keyframes.size(), "'key_frame_ind < _keyframes.size()' should hold ");

  return _pose_distance(curr_pose, _keyframe_poses[key_frame_ind]) >= 1.0;
}

int VelodyneOdom::_find_closest_keyframe(const Eigen::Matrix4d &curr_pose){
  double best_keyframe_dist = 9999999;
  int    best_keyframe_ind  = -1;
  for(int i = _keyframe_poses.size() - 1; i >= 0 ; i--){
    double curr_keyframe_dist = _pose_distance(curr_pose, _keyframe_poses[i]);
    if(curr_keyframe_dist < best_keyframe_dist){
      best_keyframe_dist = curr_keyframe_dist;
      best_keyframe_ind  = i;
    }
  }
  return best_keyframe_ind;	
}

int VelodyneOdom::_batch_optimize(int num_keyframes){
  vector<int> keyframe_indices;
  for(int i = _keyframes.size() - num_keyframes ; i < (int)_keyframes.size() ; i++)
    keyframe_indices.push_back(i);
  return _batch_optimize(keyframe_indices);
}

int VelodyneOdom::_batch_optimize(int start_keyframe_ind, int end_keyframes_ind){
  vector<int> keyframe_indices;
  for(int i = start_keyframe_ind ; i < end_keyframes_ind ; i++)
    keyframe_indices.push_back(i);
  return _batch_optimize(keyframe_indices);
}

int VelodyneOdom::_batch_optimize(const vector<int> &keyframe_indices){
  // Calculate the pose-pose distance of all of the keyframes.
  // Starting from the closest pair, re-align all the keyframes with the
  // order as they appear in the sorted distance list.
  if(keyframe_indices.size() <= 1)
    return 0;
  /*
     for(int i = 0 ; i < (int)keyframe_indices.size() ; i++){
     double neigh_dist = 9999;
     double neigh_ind  = -1;

     for(int j = 0 ; j < (int)keyframe_indices.size() ; j++){
     if(j <= i)
     continue;
     double temp_dist = _pose_distance(_keyframe_poses[i], _keyframe_poses[j], 3, DEG2RAD(10));
     if(temp_dist < neigh_dist){
     neigh_dist = temp_dist;
     neigh_ind  = j;
     }
     }

     if(neigh_dist > 1)
     continue;

     _batch_ndt.setInputTarget(_keyframes[i]);
     _batch_ndt.align(*_keyframes[neigh_ind], _keyframe_poses[neigh_ind].cast<float>());

     Eigen::Matrix4d dpose = _batch_ndt.getFinalTransformation().cast<double>();

     double dtrans = dpose.topRightCorner<3, 1>().norm();
     Eigen::Vector3d aaxis = utils::trans::dcm2aaxis(dpose.topLeftCorner<3, 3>());
     double dangle = aaxis.norm();
   */
  /*
     vector<double> dtrans, dangles;
     for(int j = i ; j < neigh_ind ; j++){

     }

     for(int j = neigh_ind ; j < (int)_keyframe.size() ; j++){

     }
     }
   */
}

int VelodyneOdom::_cancel_batch_optimization(bool undo_changes){
  return 0;
}

int VelodyneOdom::push_pc(const sensor_msgs::PointCloud2 &pc){
  pcl::fromROSMsg(pc, *_pc);	
  // If '_map' is empty and/or '_key_frame' is not set, copy.
  if(_map->size() == 0)
    _map = _pc->makeShared();
  if(_keyframes.empty()){
    _keyframes.push_back(_pc->makeShared());
    _keyframe_poses.push_back(Eigen::Matrix4d::Identity());

    _gtsam_graph_with_prior.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(_keyframe_poses[0]), _gtsam_prior_model));
  }
  return 0;
}

int VelodyneOdom::align(){
  Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
  return align(init_pose);
}

int VelodyneOdom::align(const Eigen::Vector3d &init_pos){
  Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
  init_pose.topRightCorner<3, 1>() = init_pos;
  return align(init_pose);
}

int VelodyneOdom::align(const Eigen::Matrix3d &init_dcm){
  Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
  init_pose.topLeftCorner<3, 3>() = init_dcm;
  return align(init_pose);
}

int VelodyneOdom::align(const Eigen::Matrix4d &init_pose){

  // Downsample the input point cloud using voxel grids
  _approximate_voxel_filter.setInputCloud(_pc);
  _approximate_voxel_filter.filter(*_filtered_pc);

  // Find the closest keyframe with respect to which the odometry is calculated
  _curr_keyframe_ind = _find_closest_keyframe(init_pose);

  // Align the input point cloud to the keyframe
  _ndt.setInputSource(_filtered_pc);
  _ndt.setInputTarget(_keyframes[_curr_keyframe_ind]);
  _ndt.align(*_aligned_pc, init_pose.cast<float>());

  // Check if the alignment result is sufficiently good
  //std::cout << "Normal Distributions Transform has converged:" << _ndt.hasConverged ()
  //		  << " score: " << _ndt.getFitnessScore () << std::endl;

  // Record the final robot state
  _pc_pose = _ndt.getFinalTransformation().cast<double>();

  // Add new keyframes is alignment is succesful and current pose farther from 
  // the keyframe than a threshold 
  if(_ndt.hasConverged() && _push_new_keyframe_required(_pc_pose, _curr_keyframe_ind)){
    // Use the original (non-downsampled point cloud)
    pcl::transformPointCloud(*_pc, *_aligned_pc, _ndt.getFinalTransformation());
    
    // ### *_map += *_aligned_pc; // We need to be more careful about building the map
    // Add new keyframe and gtsam factor
    _keyframes.push_back(_aligned_pc->makeShared());
    _keyframe_poses.push_back(_pc_pose);
    
    Eigen::Matrix4d dpose = _pc_pose * _keyframe_poses[_curr_keyframe_ind].inverse();
    //dpose.topRightCorner<3, 1>() = dpose.topLeftCorner<3, 3>().transpose() * dpose.topRightCorner<3, 1>();

    _gtsam_graph_with_prior.add(gtsam::BetweenFactor<gtsam::Pose3>(
          _curr_keyframe_ind, (int)_keyframes.size() - 1,
          gtsam::Pose3(dpose),
          _gtsam_prior_model));

    // Check if the robot is close to a keyframe visited earlier, "Loop Closure!!!"
    if(((int)_keyframes.size() - 1) - _curr_keyframe_ind > 1){
      gtsam::Values initial;
      for(int i = 0 ; i < (int)_keyframes.size() ; i++)
        initial.insert(i, gtsam::Pose3(_keyframe_poses[i]));

      gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(_gtsam_graph_with_prior, initial).optimize();
      for(int i = 0 ; i < (int)_keyframe_poses.size() ; i++){
        gtsam::Pose3 pose = result.at<gtsam::Pose3>(i);
        cout << "Keyframe Pose[" << i << "] before : " << endl << _keyframe_poses[i] << endl;
        _keyframe_poses[i].topLeftCorner<3, 3>() = pose.rotation().matrix();
        _keyframe_poses[i].topRightCorner<3, 1>() = pose.translation();
        cout << "Keyframe Pose[" << i << "] after  : " << endl << _keyframe_poses[i] << endl;
      }
      //exit(0);
    }

  }
  return _ndt.hasConverged() ? 0 : -1;
}

