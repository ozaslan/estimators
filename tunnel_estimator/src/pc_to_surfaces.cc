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

  TIC(__func__);
  _segment_triad_map.clear();
  _segment_origin_map.clear();
  _segment_contour_map.clear();

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

  TOC(__func__);
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
  TIC("2");
  //pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(_params.voxel_leaf_size);
  pcl::search::KdTree<pcl::PointXYZ> octree(false);
  octree.setInputCloud (_pc_sphere);
  //octree.addPointsFromInputCloud ();
  int num_pts = _pc_sphere->points.size();
  _nearest_neigh_inds.resize(num_pts);
  _nearest_neigh_sq_dists.resize(num_pts);
  _normal_covs.resize(num_pts);
  _normal_centroids.resize(num_pts);
  _normal_eigenpairs.resize(num_pts);
  _normal_min_eigval_ind.resize(num_pts);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  Eigen::Vector3d::Index min_eval_ind;
  for(int i = 0 ; i < (int)_pc_sphere->points.size() ; i++){
    octree.radiusSearch (i, _params.normal_search_radius, _nearest_neigh_inds[i], _nearest_neigh_sq_dists[i], 0);
    pcl::computeMeanAndCovarianceMatrix(*_pc_sphere, _nearest_neigh_inds[i], _normal_covs[i], _normal_centroids[i]);
    es.compute(_normal_covs[i]);
    _normal_eigenpairs[i].first = es.eigenvalues();
    _normal_eigenpairs[i].second = es.eigenvectors();
    es.eigenvalues().minCoeff(&min_eval_ind);
    _normal_min_eigval_ind[i] = min_eval_ind;
    _pc_sphere_normals->points[i].normal_x = _normal_eigenpairs[i].second(0, min_eval_ind);
    _pc_sphere_normals->points[i].normal_y = _normal_eigenpairs[i].second(1, min_eval_ind);
    _pc_sphere_normals->points[i].normal_z = _normal_eigenpairs[i].second(2, min_eval_ind);
    
  }
  TOC("2");

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
    _segment_origin_map[seg] += _segment_triad_map[seg].transpose() * dcenter;
  }

  //cout << "Triad[" << seg << "] = " << _segment_triad_map[seg] << endl;
  //cout << "Contour[" << seg << "] = " << _segment_contour_map[seg] << endl;

  return !success;
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
    _pc_projections[i] = triad * (_pc_projections[i] - origin);
    num_projected_pts++;
  }

  return num_projected_pts;
}

int PC2Surfaces::_fit_contour(int seg){
  if(_params.contour_type == "circle"){
    int num_pts = 0;
    for(int i = 0 ; i < (int)_segment_ids.size() ; i++)
      num_pts += _segment_ids[i] == seg && !_outliers[i];

    Eigen::MatrixXd A(num_pts, 3);
    Eigen::VectorXd b(num_pts);
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

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Eigen::Matrix3d(normals.transpose() * normals));

  //cout << "\n3-by-3 Matrix : " << Eigen::Matrix3d(normals.transpose() * normals) << endl;

  Eigen::Vector3d::Index min_eval_ind;
  es.eigenvalues().minCoeff(&min_eval_ind);

  //cout << "\nEigenvalues[" << seg << "] = " << es.eigenvalues() << endl;
  //cout << "\nEigenvectors[" << seg << "] = " << es.eigenvectors() << endl;

  Eigen::Matrix3d triad;

  triad.col(0) = es.eigenvectors().col(min_eval_ind);
  if(triad(0, 0) != 0)
    triad.col(0) *= SGN(triad(0,0));
  triad.col(1) << 0, 0, 1;
  triad.col(1) = triad.col(0).cross(triad.col(1));
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

    //cout << __func__ << " : " << endl << _segment_triad_map[seg] << endl;

    _viewer->addArrow(pt2, pt1, r, g, b, false, "arrow_seg" + to_string(seg));

    //_viewer->addCoordinateSystem (1, const Eigen::Affine3f &t, "ref_seg" + to_string(seg));
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
    _viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (pc_rgb, _pc_sphere_normals, 1, 0.05, "pc_sphere_normals");
    _viewer->addCoordinateSystem (1.0);
    _viewer->initCameraParameters ();
  } else {
    _viewer->updatePointCloud<pcl::PointXYZRGB> (pc_rgb, pc_rgb_handler, "pc_sphere_color");
    //_viewer->updatePointCloud<pcl::PointXYZ> (_pc_sphere, "pc_sphere");
    _viewer->removePointCloud("pc_sphere_normals", 0);
    _viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (pc_rgb, _pc_sphere_normals, 3, 0.13, "pc_sphere_normals"); 
  }

  _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc_sphere_color");

  _viewer->addSphere (pcl::PointXYZ(10, 10, 10), 3);

  if(!_viewer->wasStopped ())
    _viewer->spinOnce(1);

  return 0;
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
}
