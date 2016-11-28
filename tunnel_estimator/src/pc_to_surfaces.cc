#include "pc_to_surfaces.hh"

#define DEBUG_MSGS_ON 0
#define DEBUG_MSG_TEXT (string(__func__) + " : " + to_string(__LINE__))

utils::colors::Colors colors;

namespace utils{
  std::map<string, ros::Time> __timers__;
}

PC2Surfaces::PC2Surfaces(){
  _pc_orig = NULL;
  _pc_sphere = NULL;
  _pc_sphere_normals = NULL;
  _viewer == NULL;
}

int PC2Surfaces::push_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc){
  //TIC(__func__);
  _segment_triad_map.clear();
  _segment_origin_map.clear();
  _segment_contour_map.clear();

  _segment_Mmatrix.clear();
  _axis_eigenpairs.clear();
  _axis_min_eigval_ind.clear();

  _axis_uncertainties.clear();
  _segment_contour_equ.clear();
  _segment_dcontour.clear();
  _contour_uncertainties.clear();

  if(_pc_orig == NULL)
    _pc_orig = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(pc);
  double &vls = _params.voxel_leaf_size;
  sor.setLeafSize (vls, vls, vls);
  sor.filter (*_pc_orig);

  if(_pc_sphere == NULL)
    _pc_sphere = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  _filter_segment(_params.unclassified_id);
  _fit_normals();

  _init_triad(_params.unclassified_id);
  _segment_origin_map[_params.unclassified_id].setZero();

  //cout << "Initial Triad  : " << endl << _segment_triad_map[_params.unclassified_id] << endl;
  //cout << "Initial Origin : " << endl << _segment_origin_map[_params.unclassified_id] << endl;

  _filter_segment(0);
  _fit_segment(0);
  //cout << "Triad[0]   : " << endl << _segment_triad_map[0] << endl;
  //cout << "Origin[0]  : " << endl << _segment_origin_map[0] << endl;
  //cout << "Contour[0] : " << endl << _segment_contour_map[0] << endl;

  int num_segments = std::min(_params.num_segments, 
      (int)std::round(_params.sphere_r / _params.segment_len - 2));
  for(int seg = 1 ; seg <= num_segments ; seg++){
    _filter_segment(seg);
    _fit_segment(seg);
    //cout << "Triad[" << seg << "] : " << endl << _segment_triad_map[seg] << endl;
    //cout << "Origin[" << seg << "] : " << endl << _segment_origin_map[seg] << endl;
    _filter_segment(-seg);
    _fit_segment(-seg);
  }

  _valid_segs.clear();
  _valid_segs.push_back(0);
  for(int seg = 0 ; true ; seg++){
    auto it0 = _segment_origin_map.find(seg);
    auto it1 = _segment_origin_map.find(seg + 1);
    if(it0 != _segment_origin_map.end() && it1 != _segment_origin_map.end()){
      if( (it0->second - it1->second).norm() < _params.max_segment_dist * _params.segment_len){
        auto triad0 = _segment_triad_map[seg];
        auto triad1 = _segment_triad_map[seg + 1];
        if(utils::trans::dcm2aaxis(triad0.transpose() * triad1).norm() < _params.max_segment_rot)
          _valid_segs.push_back(seg + 1);
        else
          break;
      } else
        break;
    } else
      break;
  }

  for(int seg = 0 ; true ; seg--){
    auto it0 = _segment_origin_map.find(seg);
    auto it1 = _segment_origin_map.find(seg - 1);
    if(it0 != _segment_origin_map.end() && it1 != _segment_origin_map.end()){
      if( (it0->second - it1->second).norm() < _params.max_segment_dist * _params.segment_len){
        auto triad0 = _segment_triad_map[seg];
        auto triad1 = _segment_triad_map[seg - 1];
        if(utils::trans::dcm2aaxis(triad0.transpose() * triad1).norm() < _params.max_segment_rot)
          _valid_segs.push_back(seg - 1);
        else
          break;
      } else
          break;
    } else
      break;
  }

  //TOC(__func__);
  return _segment_triad_map.size();
}

int PC2Surfaces::_fit_normals(){

  // Compute the features
  if(_pc_sphere_normals == NULL)  
    _pc_sphere_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  _pc_sphere_normals->points.resize(_pc_sphere->points.size());

  // ------------------------------------------------------------------------------------------- //
  /*
     TIC("1");
     _ne.setInputCloud (_pc_sphere);
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
     _ne.setRadiusSearch (_params.normal_search_radius);
     _ne.setSearchMethod (tree);
     _ne.compute (*_pc_sphere_normals);
     TOC("1");
     */
  // ------------------------------------------------------------------------------------------- //
  //pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(_params.voxel_leaf_size);
  pcl::search::KdTree<pcl::PointXYZ> octree(false);
  octree.setInputCloud (_pc_sphere);
  //octree.addPointsFromInputCloud ();
  int num_pts = _pc_sphere->points.size();

  _point_uncertainties.resize(num_pts);
  // Define point uncertainties
  for(int i = 0 ; i < num_pts ; i++){
    pcl::PointXYZ &pt = _pc_sphere->points[i];
    double r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    double elevation = acos(pt.z / r);
    double azimuth   = atan2(pt.y, pt.x);
    double cos_e = cos(elevation);
    double sin_e = sin(elevation);
    double sin_a = sin(azimuth);
    double cos_a = cos(azimuth);
    Eigen::Matrix3d rot, cov;
    rot << cos_e * cos_a, cos_e * - sin_a, - sin_e,
        sin_a,           cos_a,       0,
        sin_e * cos_a, sin_e * - sin_a,   cos_e;
    cov << r * _params.var_r,                        0,                         0,
        0, r * _params.var_azimutal,                         0,
        0,                        0, r * _params.var_elevation;

    _point_uncertainties[i] = rot.transpose() * cov * rot;
  }

  _nearest_neigh_inds.resize(num_pts);
  _nearest_neigh_sq_dists.resize(num_pts);
  _normal_covs.resize(num_pts);
  _normal_centroids.resize(num_pts);
  _normal_eigenpairs.resize(num_pts);
  _normal_min_eigval_ind.resize(num_pts);
  _normal_uncertainties.resize(num_pts);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  Eigen::Vector3d::Index min_eval_ind;

  //TIC("4");
  for(int i = 0 ; i < (int)_pc_sphere->points.size() ; i++){
    octree.radiusSearch (i, _params.normal_search_radius, _nearest_neigh_inds[i], _nearest_neigh_sq_dists[i], 0);
    pcl::computeMeanAndCovarianceMatrix(*_pc_sphere, _nearest_neigh_inds[i], _normal_covs[i], _normal_centroids[i]);
    es.compute(_normal_covs[i]);
    _normal_eigenpairs[i].first = es.eigenvalues();
    _normal_eigenpairs[i].second = es.eigenvectors();
    es.eigenvalues().minCoeff(&min_eval_ind);
    _normal_min_eigval_ind[i] = min_eval_ind;

    if(_normal_eigenpairs[i].second(0, min_eval_ind) * _pc_sphere->points[i].x +
        _normal_eigenpairs[i].second(1, min_eval_ind) * _pc_sphere->points[i].y +
        _normal_eigenpairs[i].second(2, min_eval_ind) * _pc_sphere->points[i].z > 0)
      _normal_eigenpairs[i].second *= -1;

    _pc_sphere_normals->points[i].normal_x = _normal_eigenpairs[i].second(0, min_eval_ind);
    _pc_sphere_normals->points[i].normal_y = _normal_eigenpairs[i].second(1, min_eval_ind);
    _pc_sphere_normals->points[i].normal_z = _normal_eigenpairs[i].second(2, min_eval_ind);

    double min_eval = _normal_eigenpairs[i].first(min_eval_ind) ;
    Eigen::Vector3d min_evec = _normal_eigenpairs[i].second.col(min_eval_ind);
    Eigen::Matrix3d temp =  (min_eval * Eigen::Matrix3d::Identity() - _normal_covs[i]);
    temp = (temp.transpose() * temp).inverse() * temp.transpose();
    Eigen::Matrix3d dCx, dCy, dCz;

    int num_neigh_pts = _nearest_neigh_inds[i].size();

    _normal_uncertainties[i].setZero();
    Eigen::Matrix3d dnormal;
    Eigen::Vector3d pt;
    for(int j = 0 ; j < num_neigh_pts ; j++){
      dCx.setZero(); 
      dCy.setZero(); 
      dCz.setZero();

      pcl::PointXYZ &pt_pcl = _pc_sphere->points[_nearest_neigh_inds[i][j]];

      pt << pt_pcl.x, pt_pcl.y, pt_pcl.z;

      dCx.row(0) += _normal_centroids[i].topLeftCorner<3, 1>();
      dCx.col(0) += 1.0 / num_neigh_pts * pt;
      dCy.row(1) += _normal_centroids[i].topLeftCorner<3, 1>();
      dCy.col(1) += 1.0 / num_neigh_pts * pt;
      dCz.row(2) += _normal_centroids[i].topLeftCorner<3, 1>();
      dCz.col(2) += 1.0 / num_neigh_pts * pt;

      dnormal.col(0) = temp * dCx * min_evec / num_neigh_pts;
      dnormal.col(1) = temp * dCy * min_evec / num_neigh_pts;
      dnormal.col(2) = temp * dCz * min_evec / num_neigh_pts;

      _normal_uncertainties[i] += dnormal.transpose() * _point_uncertainties[_nearest_neigh_inds[i][j]] * dnormal;
    }
  }
  //TOC("4");

  return _pc_sphere_normals->size();
}

int PC2Surfaces::_filter_segment(int seg){
  Eigen::Matrix3d triad;
  Eigen::Vector3d origin;

  if(seg == _params.unclassified_id){
    int num_pts = _pc_orig->size();
    _pc_sphere->clear();
    _pc_sphere->reserve(num_pts / 3);
    double r_squared = pow(_params.sphere_r, 2);
    for(int i = 0 ; i < num_pts ; i++){
      pcl::PointXYZ &pt = (*_pc_orig)[i];
      if(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= r_squared)
        _pc_sphere->push_back(pt);
    }
    _segment_ids.clear();
    _segment_ids.resize(_pc_sphere->size(), _params.unclassified_id);
    _outliers.clear();
    _outliers.resize(_pc_sphere->size(), false);

    return seg;

  } else if(seg == 0){
    triad  = _segment_triad_map[_params.unclassified_id];
    origin = _segment_origin_map[_params.unclassified_id];
  } else {
    // Check if triad and origin for 'seg' is already initialized
    auto it = _segment_origin_map.find(seg);
    if(it != _segment_origin_map.end()){
      triad = _segment_triad_map[seg];
      origin = it->second;
    } else {
      triad  =  _segment_triad_map[seg - SGN(seg)];
      origin = _segment_origin_map[seg - SGN(seg)];
      origin = origin + SGN(seg) * _params.segment_len * triad.topLeftCorner<3, 1>(); 
    }
  }

  int num_pts = _pc_sphere->size();
  double upper_mult, lower_mult;
  upper_mult = seg < 0 ? 1.3 : 1;
  lower_mult = seg > 0 ? 1.3 : 1;
  for(int i = 0 ; i < num_pts ; i++){
    pcl::PointXYZ &pcl_pt = (*_pc_sphere)[i];
    Eigen::Vector3d pt;
    pt << pcl_pt.x, pcl_pt.y, pcl_pt.z;
    pt = triad.transpose() * (pt - origin);
    if(-_params.segment_len / 2 * lower_mult < pt(0) && pt(0) <= _params.segment_len / 2 * upper_mult)
      if(_segment_ids[i] == _params.unclassified_id)
        _segment_ids[i] = seg;
  }

  return seg;
}

int PC2Surfaces::_fit_segment(int seg){

  bool success = false;

  for(int outer_iter = 0 ; !success && outer_iter < _params.max_outer_iter ; outer_iter++){
    //cout << "outer_iter : " << outer_iter << endl;
    success = false;
    for(int inner_iter = 0 ; !success && inner_iter < _params.max_inner_iter ; inner_iter++){
      //cout << "inner_iter 1 : " << inner_iter << endl;
      _init_triad(seg);
      success = !_eliminate_outliers(seg, "normals");
    }

    _project_pc(seg);

    success = false;
    for(int inner_iter = 0 ; !success && inner_iter < _params.max_inner_iter ; inner_iter++){
      //cout << "inner_iter 2 : " << inner_iter << endl;
      _fit_contour(seg);
      success = !_eliminate_outliers(seg, "contour");
    }
  }

  if(success){
    Eigen::Vector3d contour = _segment_contour_map[seg];
    Eigen::Vector3d dcenter(0, contour(0), contour(1));
    _segment_origin_map[seg] += _segment_triad_map[seg] * dcenter;

    _estimate_uncertainties(seg);
  }

  //cout << "Triad[" << seg << "] = " << _segment_triad_map[seg] << endl;
  //cout << "Contour[" << seg << "] = " << _segment_contour_map[seg] << endl;

  return !success;
}

int PC2Surfaces::_estimate_uncertainties(int seg){

  TIC(__func__);
  Eigen::Matrix3d dMx, dMy, dMz, daxis;
  dMx.setZero();
  dMy.setZero();
  dMz.setZero();

  int min_eval_ind = _axis_min_eigval_ind[seg];
  double min_eval = _axis_eigenpairs[seg].first(min_eval_ind) ;
  Eigen::Vector3d min_evec = _axis_eigenpairs[seg].second.col(min_eval_ind);
  Eigen::Matrix3d temp =  (min_eval * Eigen::Matrix3d::Identity() - _segment_Mmatrix[seg]);
  temp = (temp.transpose() * temp).inverse() * temp.transpose();

  int num_pts = _pc_sphere->points.size();

  Eigen::MatrixXd &A = _segment_contour_equ[seg].first;
    if(A.rows() == 0)
      return -1;
  Eigen::VectorXd &b = _segment_contour_equ[seg].second;

  JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd &U = svd.matrixU();
  const Eigen::MatrixXd &V = svd.matrixV();
  Eigen::Matrix3d D;
  D.setZero();
    //cout << A << endl;
  D.diagonal() = svd.singularValues();
  Eigen::Matrix3d D_inv = D.inverse();
  Eigen::Matrix3d D_inv_sqr = D_inv * D_inv;

  Eigen::MatrixXd dU;
  Eigen::Matrix3d dD, dV;
  Eigen::VectorXd db;
  Eigen::Matrix<double, 3, 2> dX;

  dU = Eigen::MatrixXd::Zero(U.rows(), U.cols());
  db = Eigen::VectorXd::Zero(b.rows());
  dV.setZero();
  dD.setZero();

  Eigen::Matrix3d omegaU, omegaV;
  omegaU.setZero();
  omegaV.setZero();

  const Eigen::Matrix3d &triad = _segment_triad_map[seg];

  Eigen::Matrix3d err;

  _contour_uncertainties[seg] = Eigen::MatrixXd(3, 3);
  _contour_uncertainties[seg].setZero();
  Eigen::MatrixXd &contour_uncertainty = _contour_uncertainties[seg];

  Eigen::MatrixXd Temp01 = V * D_inv;
  Eigen::MatrixXd Temp02 = D_inv * U.transpose();
  Eigen::MatrixXd Temp03 = U.transpose() * b;
  //Eigen::MatrixXd Temp04 = Temp01 * U.transpose();
  Eigen::MatrixXd Temp05 = D_inv_sqr * Temp03;
  Eigen::MatrixXd Temp06 = Temp02 * b;
  Eigen::MatrixXd Temp07 = V * Temp02;


  for(int i = 0, j = 0 ; i < num_pts ; i++){
    if(_outliers[i] == false && _segment_ids[i] == seg){
      pcl::Normal &normal_pcl = _pc_sphere_normals->points[i];
      Eigen::Vector3d normal;
      normal << normal_pcl.normal_x, normal_pcl.normal_y, normal_pcl.normal_z;

      dMx.row(0) += normal;
      dMx.col(0) += normal;
      dMy.row(1) += normal;
      dMy.col(1) += normal;
      dMz.row(2) += normal;
      dMz.col(2) += normal;

      daxis.col(0) = temp * dMx * min_evec;
      daxis.col(1) = temp * dMy * min_evec;
      daxis.col(2) = temp * dMz * min_evec;

      _axis_uncertainties[seg] += daxis * _normal_uncertainties[i] * daxis.transpose();

      // ------------------------------------------------------- //
      // Project point uncertaincy
      Eigen::MatrixXd pt_uncertainty = triad * _point_uncertainties[i] * triad.transpose();
      pt_uncertainty = pt_uncertainty.bottomRightCorner<2, 2>();

      for(int dim = 0 ; dim <= 1 ; dim++){

        // Calculate dD
        err = U.row(j).transpose() * V.row(dim);
        dD.diagonal() = err.diagonal();  

        // Calculate dV & dU
        Eigen::Matrix2d temp;
        Eigen::Vector2d uv, soln;
        for(int k = 0 ; k < 3 ; k++){
          for(int l = k + 1 ; l < 3 ; l++){
            temp << D(l, l), D(k, k), D(k, k), D(l, l);
            uv << err(k, l), -err(l, k);
            soln = temp.inverse() * uv;
            omegaU(k, l) =  soln(0);
            omegaU(l, k) = -soln(0);
            omegaV(k, l) =  soln(1);
            omegaV(l, k) = -soln(1);
          }
        }
        dU =  U * omegaU;
        dV = -V * omegaV;

        // Calculate db
        db.setZero();

        db(j) = -2 * _pc_projections[i](dim);

        // Calculate dX

        dX.col(dim) = 
          Temp01 * dU.transpose() *  b +
          V * dD * Temp05 +
          dV * Temp06 +
          Temp07 * db;

      }

      // Calculate dxc, dyc, dR
      Eigen::MatrixXd &dcontour = _segment_dcontour[seg];
      Eigen::MatrixXd temp = dcontour * dX;
      contour_uncertainty += temp * pt_uncertainty * temp.transpose();

      j++;
    }
  }
  //cout << "Contour Uncert[" << seg << "] = " << endl << _contour_uncertainties[seg] << endl;
}


int PC2Surfaces::_project_pc(int seg){
  int num_pts = _pc_sphere->size();

  Eigen::Matrix3d triad  = _segment_triad_map[seg];
  Eigen::Vector3d origin = _segment_origin_map[seg];

  if(_pc_projections.size() != _pc_sphere->size())
    _pc_projections.resize(_pc_sphere->size());

  int num_projected_pts = 0;
  for(int i = 0 ; i < num_pts ; i++){
    if(_segment_ids[i] != seg)
      continue;
    pcl::PointXYZ pt = (*_pc_sphere)[i];
    _pc_projections[i] << pt.x, pt.y, pt.z;
    _pc_projections[i] = triad.transpose() * (_pc_projections[i] - origin);
    num_projected_pts++;
  }

  return num_projected_pts;
}

int PC2Surfaces::_fit_contour(int seg){
  if(_params.contour_type == "circle"){
    int num_pts = 0;
    for(int i = 0 ; i < (int)_segment_ids.size() ; i++)
      num_pts += _segment_ids[i] == seg && !_outliers[i];

    _segment_contour_equ[seg].first  = Eigen::MatrixXd(num_pts, 3);
    _segment_contour_equ[seg].second = Eigen::VectorXd(num_pts);

    Eigen::MatrixXd &A = _segment_contour_equ[seg].first;
    Eigen::VectorXd &b = _segment_contour_equ[seg].second;

    num_pts = _pc_projections.size();
    for(int i = 0, j = 0 ; i < num_pts ; i++){
      if(_segment_ids[i] == seg && !_outliers[i]){
        double y, z;
        y = _pc_projections[i](1);
        z = _pc_projections[i](2);
        A.row(j) << y, z, 1;
        b(j) = -(y * y + z * z);
        j++;
      }
    }

    Eigen::VectorXd soln = (A.transpose() * A).inverse() * A.transpose() * b; 
    double yc = - soln(0) / 2;
    double zc = - soln(1) / 2;
    double R  = sqrt((soln(0) * soln(0) + soln(1) * soln(1)) / 4 - soln(2));

    _segment_contour_map[seg].resize(3, 1);
    _segment_contour_map[seg] << yc, zc, R; 

    _segment_dcontour[seg].resize(3, 3);
    _segment_dcontour[seg].row(0) << -0.5, 0, 0;
    _segment_dcontour[seg].row(1) << 0, -0.5, 0;
    _segment_dcontour[seg].row(2) << yc / (4 * R), zc / (4 * R), -0.5 / R ;

    //cout << "Contour fit [" << seg <<"] : " << xc << ", " << yc << ", " << R << endl;

    return 3;
  } else {
    return 0;
  }
}

int PC2Surfaces::_init_triad(int seg){

  int num_pts = 0;

  for(int i = 0 ; i < (int)_segment_ids.size() ; i++)
    num_pts += _segment_ids[i] == seg && !_outliers[i];

  Eigen::MatrixXd normals(num_pts, 3);

  Eigen::Vector3d origin(0, 0, 0);

  Eigen::Vector3d init_axis_estimate;
  if(seg == 0)
    init_axis_estimate = _segment_triad_map[_params.unclassified_id].topLeftCorner<3, 1>();
  else
    init_axis_estimate = _segment_triad_map[seg - SGN(seg)].topLeftCorner<3, 1>();

  //cout << "Initial Triad[" << seg << "] : " << endl << init_axis_estimate << endl;

  num_pts = _pc_sphere->size();
  int num_valid_normals = 0;
  int num_points = 0;
  for(int i = 0, j = 0 ; i < num_pts ; i++){
    if(_segment_ids[i] != seg || _outliers[i])
      continue;
    pcl::Normal n = _pc_sphere_normals->points[i];
    if(isnan(n.normal_x) || isnan(n.normal_y) || isnan(n.normal_z))
      normals.row(j++).setZero();
    else {
      normals(j, 0) = n.normal_x;
      normals(j, 1) = n.normal_y;
      normals(j, 2) = n.normal_z;
      double curv_mult = exp(-pow(n.curvature / _params.curvature_thres, 2));
      if(seg == _params.unclassified_id)
        normals.row(j) *= curv_mult;
      else
        normals.row(j) *= (1 - fabs(normals.row(j).dot(init_axis_estimate))) * curv_mult;
      j++;
      num_valid_normals++;
    }

    origin(0) += (*_pc_sphere)[i].x;
    origin(1) += (*_pc_sphere)[i].y;
    origin(2) += (*_pc_sphere)[i].z;
    num_points++;
  }

  _segment_origin_map[seg] = origin / num_points;

  //cout << "normals[" << seg << "] = " << normals << endl;

  _segment_Mmatrix[seg] = normals.transpose() * normals;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(_segment_Mmatrix[seg]);

  //cout << "\n3-by-3 Matrix : " << Eigen::Matrix3d(normals.transpose() * normals) << endl;

  Eigen::Vector3d::Index min_eval_ind;
  es.eigenvalues().minCoeff(&min_eval_ind);

  _axis_eigenpairs[seg].first = es.eigenvalues();
  _axis_eigenpairs[seg].second = es.eigenvectors();
  _axis_min_eigval_ind[seg] = min_eval_ind;

  //cout << "\nEigenvalues[" << seg << "] = " << es.eigenvalues() << endl;
  //cout << "\nEigenvectors[" << seg << "] = " << es.eigenvectors() << endl;

  Eigen::Matrix3d triad;

  triad.col(0) = es.eigenvectors().col(min_eval_ind);
  if(triad(0, 0) != 0)
    triad.col(0) *= SGN(triad(0,0));
  triad.col(1) << 0, 0, 1;
  triad.col(1) = -triad.col(0).cross(triad.col(1));
  triad.col(1) /= triad.col(1).norm();
  triad.col(2) = triad.col(0).cross(triad.col(1));

  //cout << "\nTriad[" << seg << "] = " << triad << endl;

  _segment_triad_map[seg] = triad;

  return num_valid_normals;
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
        normal << pcl_normal.normal_x, pcl_normal.normal_y, pcl_normal.normal_z;
        double dot_prod = normal.dot(segment_axis);
        if(fabs(acos(dot_prod) - M_PI/2) > DEG2RAD(_params.normal_dev_thres2)){
          num_eliminated += _outliers[i] == false;
          _outliers[i] = true;
        } else
          1;//_outliers[i] = false;
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
            //_segment_ids[i] = _params.outlier_id;
            num_eliminated += _outliers[i] == false;
            _outliers[i] = true;
          } else
            1;//_outliers[i] = false;
        }
      }
    }
  }
  //cout << "num_eliminated[" << seg << "] : " << num_eliminated << endl;
  return num_eliminated;
}

int PC2Surfaces::visualize_fit(){

  //---------------------------------------------------------//
  // -----Open 3D viewer and add point cloud and normals-----//
  // --------------------------------------------------------//
  bool is_first_frame = _viewer == NULL;
  if(is_first_frame)
    _viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));

  _viewer->setBackgroundColor (0.3, 0.3, 0.3);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*_pc_sphere, *pc_rgb);

  // Colorify the point cloud according to segment ids
  for(int i = 0 ; i < (int)_segment_ids.size() ; i++){
    int seg = _segment_ids[i];
    if(seg == _params.unclassified_id)
      seg = 1;
    else if(seg == _params.outlier_id)
      seg = 0;
    else
      seg = 2 * abs(seg) - (SGN(seg) < 0 ? 1 : 0) + 2;
    Eigen::Vector3d color = colors[seg];
    int r(255 * color(0)), g(255 * color(1)), b(255 * color(2));

    if(_outliers[i] == true)    
      r = g = b = 0;

    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
        static_cast<uint32_t>(g) << 8  |
        static_cast<uint32_t>(b));
    (*pc_rgb)[i].rgb = *reinterpret_cast<float*>(&rgb);
  }

  _viewer->removeAllShapes();

  for(auto it : _segment_triad_map) {
    int seg = it.first, color_ind;
    if(seg == _params.unclassified_id || seg == _params.outlier_id)
      continue;
    else
      color_ind = 2 * abs(seg) - (SGN(seg) < 0 ? 1 : 0) + 2;

    Eigen::Vector3d color = colors[color_ind];
    int r(255 * color(0)), g(255 * color(1)), b(255 * color(2));

    pcl::PointXYZ pt1, pt2;
    pt1.x = _segment_origin_map[seg](0);
    pt1.y = _segment_origin_map[seg](1);
    pt1.z = _segment_origin_map[seg](2);
    pt2.x = pt1.x + _segment_triad_map[seg](0, 0) * 1;
    pt2.y = pt1.y + _segment_triad_map[seg](1, 0) * 1;
    pt2.z = pt1.z + _segment_triad_map[seg](2, 0) * 1;

    if(!isfinite(pt1.x + pt1.y + pt1.z + pt2.x + pt2.y + pt2.z) ||
        fabs(pt1.x) > 100 ||
        fabs(pt1.y) > 100 ||
        fabs(pt1.z) > 100 ||
        fabs(pt2.x) > 100 ||
        fabs(pt2.y) > 100 ||
        fabs(pt2.z) > 100)
      continue;

    _viewer->addArrow(pt2, pt1, r, g, b, false, "arrow_seg" + to_string(seg));
  }

  _viewer->removeAllCoordinateSystems();

  for(auto it : _segment_contour_map){
    int seg = it.first;
    Eigen::VectorXd fit = it.second;
    Eigen::Matrix3d triad = _segment_triad_map[seg];
    Eigen::Vector3d origin = _segment_origin_map[seg];

    pcl::ModelCoefficients cylinder_coeff;
    cylinder_coeff.values.resize (7);    // We need 7 values
    cylinder_coeff.values[0] = origin(0) - _params.segment_len / 2 * triad(0, 0);
    cylinder_coeff.values[1] = origin(1) - _params.segment_len / 2 * triad(1, 0);
    cylinder_coeff.values[2] = origin(2) - _params.segment_len / 2 * triad(2, 0);
    cylinder_coeff.values[3] = _params.segment_len * triad(0, 0);
    cylinder_coeff.values[4] = _params.segment_len * triad(1, 0);
    cylinder_coeff.values[5] = _params.segment_len * triad(2, 0);
    cylinder_coeff.values[6] = fit(2);

    string id = string("cylinder") + to_string(seg); 

    _viewer->addCylinder (cylinder_coeff, id); 

    _viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, id); 
    _viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, .3, id); 
    _viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, id );

    Eigen::Affine3d affine3d;
    affine3d.fromPositionOrientationScale (origin, triad, Eigen::Vector3d::Ones());
    _viewer->addCoordinateSystem(0.5, affine3d.cast<float>(), "reference" + to_string(seg));

  }


  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pc_rgb_handler(pc_rgb);
  if(is_first_frame){
    _viewer->addPointCloud<pcl::PointXYZRGB> (pc_rgb, pc_rgb_handler, "pc_sphere_color");
    //_viewer->addPointCloud<pcl::PointXYZ> (_pc_sphere, "pc_sphere");
    _viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (pc_rgb, _pc_sphere_normals, 10, 0.05, "pc_sphere_normals");
    _viewer->addCoordinateSystem (1.0);
    _viewer->initCameraParameters ();
  } else {
    _viewer->updatePointCloud<pcl::PointXYZRGB> (pc_rgb, pc_rgb_handler, "pc_sphere_color");
    //_viewer->updatePointCloud<pcl::PointXYZ> (_pc_sphere, "pc_sphere");
    _viewer->removePointCloud("pc_sphere_normals", 0);
    _viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (pc_rgb, _pc_sphere_normals, 3, 0.13, "pc_sphere_normals"); 
  }

  _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc_sphere_color");

  if(!_viewer->wasStopped ())
    _viewer->spinOnce(1);

  return 0;
}

int PC2Surfaces::get_segment_pc(int seg, pcl::PointCloud<pcl::PointXYZ> &pc){
  pc.points.clear();
  pc.points.reserve(_pc_sphere->points.size() / 5);
  int num_pts = _segment_ids.size();
  for(int i = 0 ; i < num_pts ; i++){
    if(_segment_ids[i] == seg && _outliers[i] == false)
      pc.points.push_back(_pc_sphere->points[i]);
  }

  return pc.points.size();
}

int PC2Surfaces::get_projections(int seg, vector<Eigen::Vector3d> &proj, vector<bool> &outliers){
  int num_pts = 0;
  for(auto ind : _segment_ids)
    num_pts += ind == seg;
  proj.clear();
  outliers.clear();
  proj.reserve(num_pts);
  outliers.reserve(num_pts);
  for(int i = 0 ; i < num_pts ; i++)
    if(_segment_ids[i] == seg){
      proj.push_back(_pc_projections[i]);
      outliers.push_back(_outliers[i]);
    }
  return num_pts;
}


PC2SurfacesParams::PC2SurfacesParams(){
  max_inner_iter = 9;
  max_outer_iter = 5;
  outlier_id = -99999;
  unclassified_id = 99999;
  term_perc_crit = 0.07;
  normal_dev_thres1 = 15; // degrees
  normal_dev_thres2 = 10; 
  curvature_thres = 0.1;
  contour_fit_thres = 0.10; // ratio to radius
  sphere_r = 7.5; // meters
  segment_len = 1; // meters
  normal_search_radius = 0.25;
  contour_type = "circle";
  num_segments = 7;
  voxel_leaf_size = 0.10;
  curvature_thres = 0.10;
  var_r = 0.001;
  var_azimutal = 0.001;
  var_elevation = 0.001;
  max_segment_dist = segment_len * 1.3;
  max_segment_rot  = DEG2RAD(10);
}

void PC2SurfacesParams::print(){
  cout << "max_inner_iter ----- : " << max_inner_iter << endl;
  cout << "max_outer_iter ----- : " << max_outer_iter << endl;
  cout << "term_perc_crit ----- : " << term_perc_crit << endl;
  cout << "normal_dev_thres1 -- : " << normal_dev_thres1 << endl;
  cout << "normal_dev_thres2 -- : " << normal_dev_thres2 << endl;
  cout << "contour_fit_thres -- : " << contour_fit_thres << endl;
  cout << "normal_search_radius : " << normal_search_radius << endl;
  cout << "sphere_r ----------- : " << sphere_r << endl;
  cout << "segment_len -------- : " << segment_len << endl;
  cout << "contour_type ------- : " << contour_type << endl;
  cout << "num_segments ------- : " << num_segments << endl;
  cout << "voxel_leaf_size ---- : " << voxel_leaf_size << endl;
  cout << "curvature_thres ---- : " << curvature_thres << endl;
  cout << "max_segment_dist --- : " << max_segment_dist << endl;
  cout << "max_segment_rot ---- : " << max_segment_rot << endl;
}
