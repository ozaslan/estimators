#include "tunnel_estimator/pc_to_surfaces.hh"

#define SGN(x) ((x) == 0 ? 0 : ((x) < 0 ? -1 : +1))


PC2Surfaces::PC2Surfaces(){

}

int PC2Surface::push_pc(const pcl::PointCloud<pcl::PointXYZ>::ptr &pc){

  _pc_orig = pc;
  // Filter the original PC to only points that are at most 'r' distance from the the sensor origin
  _filter_segment(_unclassified_id);
  // Fit normals to '_pc_sphere'
  _fit_normals(_unclassified_id);
  // Estimate an initial triad using all the normals
  _estimate_triad(_unclassified_id);

  _filter_segment(0);
  _fit_segment(0);

  for(int seg = 0 ; seg <= _sphere_r/_segment_len ; seg++){
    _filter_segment(seg);
    _fit_segment(seg);
    _filter_segment(-seg);
    _fit_segment(-seg);
  }

  return 0;
}

int PC2Surfaces::_fit_normals(){

  _ne.setInputCloud (_pc_sphere);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  _ne.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  _ne.setRadiusSearch (_normal_search_radius);

  // Compute the features
  _ne.compute (*_pc_sphere_normals);
}

bool PC2Surfaces::_filter_segment(int seg){
  Eigen::Matrix3d triad;
  Eigen::Vector3d origin;

  if(seg == _unclassified_id){
    int num_pts = _pc_orig.size();
    _pc_sphere.reserve(num_pts / 3);
    for(int i = 0 ; i < num_pts ; i++){
      pcl::PointXYZ pt = _pc_origs[i];
      if(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= _sphere_r * _sphere_r)
        _pc_sphere.push_back(pt);
    }
    _segment_ids.clear();
    _segment_ids.resize(_pc_sphere.size(), _unclassified_id);

    return true;

  } else if(seg == 0){
    triad  = _segment_triad_map[_unclassified_id];
    origin = _segment_origin_map[_unclassified_id];
  } else {
    triad  =  _segment_triad_map[seg + SGN(seg)];
    origin = _segment_origin_map[seg + SGN(seg)];
    origin = origin + 2 * triad.topLeftCorner<3, 1>() * _segment_len;
  }

  int num_pts = _pc_sphere.size();
  for(int i = 0 ; i < num_pts ; i++){
    pcl::PointXYZ pt = _pc_sphere[i];
    pt = triad(pt - origin);
    if(-_segment_len < pt.x && pt.x <= _segment_len)
      _segment_ids[i] = seg;
  }

  return true;
}

bool PC2Surfaces::_fit_segment(int seg){

  bool success = false

    for(int outer_iter = 0 ; !success && outer_iter < _max_outer_iter ; outer_iter++)
      success = false;
  for(int inner_iter = 0 ; !success && inner_iter < _max_inner_iter ; inner_iter++){
    _estimate_triad(0);
    success = !_eliminate_outliers(0, "normals");
  }

  _project_pc(0);

  success = false;
  for(int inner_iter = 0 ; !success && inner_iter < _max_inner_iter ; inner_iter++){
    _fit_contour(0);
    success = i!_eliminate_outliers(0, "surface");
  }
}

return success;
}

bool PC2Surfaces::_estimate_triad(int seg){

  int num_pts = 0;
  for(int ind : _segment_ids)
    num_pts += ind == seg;

  Eigen::MatrixXd normals(num_pts, 3);

  num_pts = _pc_sphere.size();
  for(int i = 0, j = 0 ; i < num_pts ; i++){
    if(_segment_ids[i] != seg)
      continue;
    normals(j  , 0) = _pc_sphere_normals[i].x;
    normals(j  , 1) = _pc_sphere_normals[i].y;
    normals(j++, 2) = _pc_sphere_normals[i].z;
  }

  EigenSolver<MatrixXd> es(normals.transpose() * normals);

  int min_eval_ind;
  es.eigenvalues().minCoeff(min_eval_ind, NULL);

  Eigen::Matrix3d triad;

  triad.col(0) = es.eigenvectors.col(min_eval_ind);
  triad.col(1) << 0, 0, 1;
  triad.col(1) = triad.col(0).cross(triad.col(1));
  triad.col(2) = triad.col(0).cross(triad.col(1));

  _segment_triad_map[seg] = triad;
}

int PC2Surfaces::_eliminate_outliers(int seg, const std::string &method){

}
