#include "pc_to_surfaces.hh"

#define SGN(x) ((x) == 0 ? 0 : ((x) < 0 ? -1 : +1))


PC2Surfaces::PC2Surfaces(){

}

int PC2Surfaces::push_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc){

  _pc_orig = pc;
  // Filter the original PC to only points that are at most 'r' distance from the the sensor origin
  _filter_segment(_params.unclassified_id);
  // Fit normals to '_pc_sphere'
  _fit_normals();
  // Estimate an initial triad using all the normals
  _init_triad(_params.unclassified_id);

  _filter_segment(0);
  _fit_segment(0);

  for(int seg = 0 ; seg <= _params.sphere_r / _params.segment_len ; seg++){
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
  _ne.setRadiusSearch (_params.normal_search_radius);

  // Compute the features
  _ne.compute (*_pc_sphere_normals);

  return _pc_sphere_normals->size();
}

int PC2Surfaces::_filter_segment(int seg){
  Eigen::Matrix3d triad;
  Eigen::Vector3d origin;

  if(seg == _params.unclassified_id){
    int num_pts = _pc_orig->size();
    _pc_sphere->reserve(num_pts / 3);
    double r_squared = pow(_params.sphere_r, 2);
    for(int i = 0 ; i < num_pts ; i++){
      pcl::PointXYZ pt = (*_pc_orig)[i];
      if(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= r_squared)
        _pc_sphere->push_back(pt);
    }
    _segment_ids.clear();
    _segment_ids.resize(_pc_sphere->size(), _params.unclassified_id);

    return true;

  } else if(seg == 0){
    triad  = _segment_triad_map[_params.unclassified_id];
    origin = _segment_origin_map[_params.unclassified_id];
  } else {
    triad  =  _segment_triad_map[seg - SGN(seg)];
    origin = _segment_origin_map[seg - SGN(seg)];
    origin = origin + 2 * SGN(seg) * _params.segment_len * triad.topLeftCorner<3, 1>(); 
  }

  int num_pts = _pc_sphere->size();
  for(int i = 0 ; i < num_pts ; i++){
    pcl::PointXYZ pt = (*_pc_sphere)[i];
    pt.x -= origin(0);
    if(-_params.segment_len < pt.x && pt.x <= _params.segment_len)
      _segment_ids[i] = seg;
  }

  return true;
}

int PC2Surfaces::_project_pc(int seg){
  int num_pts = _pc_sphere->size();

  Eigen::Matrix3d triad  = _segment_triad_map[seg];
  Eigen::Vector3d origin = _segment_origin_map[seg];

  if(_pc_projections.size() != _pc_sphere->size())
    _pc_projections.resize(_pc_sphere->size());

  for(int i = 0 ; i < num_pts ; i++){
    if(_segment_ids[i] != seg)
      continue;
    pcl::PointXYZ pt = (*_pc_sphere)[i];
    _pc_projections[i] << pt.x, pt.y, pt.z;
    _pc_projections[i] = triad * (_pc_projections[i] - origin);
  }
}

int PC2Surfaces::_fit_segment(int seg){

  bool success = false;

  for(int outer_iter = 0 ; !success && outer_iter < _params.max_outer_iter ; outer_iter++){
    success = false;
    for(int inner_iter = 0 ; !success && inner_iter < _params.max_inner_iter ; inner_iter++){
      _init_triad(seg);
      success = !_eliminate_outliers(seg, "normals");
    }

    _project_pc(seg);

    success = false;
    for(int inner_iter = 0 ; !success && inner_iter < _params.max_inner_iter ; inner_iter++){
      _fit_contour(seg);
      success = !_eliminate_outliers(0, "contour");
    }
  }
}

int PC2Surfaces::_fit_contour(int seg){
  if(type == _params.contour_type){
    int num_pts = 0;
    for(int segment : _segment_ids)
      num_pts += segment == seg;
    Eigen::MatrixXd A(num_pts, 3);
    Eigen::VectorXd b(num_pts);
    num_pts = _pc_projections.size();
    for(int i = 0 ; i < num_pts ; i++){
      double y, z;
      y = _pc_projections[i](1);
      z = _pc_projections[i](2);
      A.row(i) << y, z, 1;
      b(i) = -(y * y + z * z);
    }

    Eigen::VectorXd soln = (A.transpose() * A).inverse() * A.transpose() * b; 
    double xc = - soln(0) / 2;
    double yc = - soln(1) / 2;
    double R  = sqrt((soln(0) * soln(0) + soln(1) * soln(1)) / 4 - soln(2));

    _segment_contour_map[seg] << xc, yc, R; 

    return 3;
  } else {
    return 0;
  }
}

int PC2Surfaces::_init_triad(int seg){

  int num_pts = 0;
  for(int ind : _segment_ids)
    num_pts += ind == seg;

  Eigen::MatrixXd normals(num_pts, 3);

  num_pts = _pc_sphere->size();
  for(int i = 0, j = 0 ; i < num_pts ; i++){
    if(_segment_ids[i] != seg)
      continue;
    pcl::Normal pc_normal = (*_pc_sphere_normals)[i];
    normals(j  , 0) = pc_normal.data_c[0];
    normals(j  , 1) = pc_normal.data_c[1];
    normals(j++, 2) = pc_normal.data_c[2];
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Eigen::Matrix3d(normals.transpose() * normals));

  Eigen::Vector3d::Index min_eval_ind;
  es.eigenvalues().minCoeff(&min_eval_ind);

  Eigen::Matrix3d triad;

  triad.col(0) = es.eigenvectors().col(min_eval_ind);
  triad.col(1) << 0, 0, 1;
  triad.col(1) = triad.col(0).cross(triad.col(1));
  triad.col(2) = triad.col(0).cross(triad.col(1));

  _segment_triad_map[seg] = triad;
}

int PC2Surfaces::_eliminate_outliers(int seg, const std::string &method){
  int num_pts = _pc_sphere->size();
  int num_eliminated = 0;
  if(method == "normals"){
    Eigen::Vector3d segment_axis = _segment_triad_map[seg].topLeftCorner<3, 1>();
    Eigen::Vector3d normal;
    for(int i = 0 ; i < num_pts ; i++){
      if(_segment_ids[i] == seg){
        pcl::Normal &pcl_normal = (*_pc_sphere_normals)[i];
        normal << pcl_normal.data_c[0], pcl_normal.data_c[1], pcl_normal.data_c[2];
        double dot_prod = normal.dot(segment_axis);
        if(acos(dot_prod) > _params.normal_dev_thres1){
          _segment_ids[i] = _params.outlier_id;
          num_eliminated++;
        }
      }
    }
  } else if(method == "contour"){
    if(_params.contour_type == "circle"){
      Eigen::VectorXd &contour_fit = _segment_contour_map[seg];
      double xc = contour_fit(0);
      double yc = contour_fit(1);
      double R  = contour_fit(2);
      for(int i = 0 ; i < num_pts ; i++){
        if(_segment_ids[i] == seg){
          Eigen::Vector3d &proj_pt = _pc_projections[i];
          double r = sqrt(pow(proj_pt(1) - xc, 2) + pow(proj_pt(2) - yc, 2));
          if( fabs(R - r) / R > _params.contour_fit_thres){
            _segment_ids[i] = _params.outlier_id;
            num_eliminated++;
          }
        }
      }
    }
  }
  return num_eliminated;
}
