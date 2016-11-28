#ifndef __PC_TO_SURFACES_HH__
#define __PC_TO_SURFACES_HH__

#include <vector>                                                                                                                                                                                              
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>


#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Eigenvalues> 
#include <Eigen/SVD>

#include <utils.hh>

struct PC2SurfacesParams{
  public:
    PC2SurfacesParams();
    void print();

    int max_inner_iter;
    int max_outer_iter;
    int outlier_id;
    int unclassified_id;
    double term_perc_crit;
    double normal_dev_thres1; // degrees
    double normal_dev_thres2; 
    double contour_fit_thres; // ratio to radius
    double sphere_r; // meters
    double segment_len; // meters
    double normal_search_radius;
    int    num_segments;
    string contour_type;
    double voxel_leaf_size;
    double curvature_thres;
    double var_r;
    double var_azimutal;
    double var_elevation;
    double max_segment_dist;
    double max_segment_rot;
};

class PC2Surfaces{
  private:
    PC2SurfacesParams _params;

    // ------------ BOOKKEEPING --------------------------------------------------- //
    // Segment ID of each point in '_pc_sphere'
    vector<int> _segment_ids;
    vector<int> _valid_segs;
    // Outlier flags for points in '_pc_sphere'
    vector<bool> _outliers;
    // Mapping between segments and their corresponding 
    // coordinate triad, origin and contour equations.
    std::map<int, Eigen::Matrix3d> _segment_triad_map;
    std::map<int, Eigen::Vector3d> _segment_origin_map;
    std::map<int, Eigen::VectorXd> _segment_contour_map;

    // ------------ AXIS, CONTOUR & UNCERTAINTY ESTIMATION ------------------------ //
    std::map<int, Eigen::Matrix3d> _segment_Mmatrix;
    // Mapping between segments and eigenpairs of the 'M' matrix as 
    // stored in '_segment_Mmatrix'.
    std::map<int, pair<Eigen::Vector3d, Eigen::Matrix3d> > _axis_eigenpairs;
    // Mapping between segments and index of the smallest eigen value
    // of the corresponding eigenpair.
    std::map<int, int> _axis_min_eigval_ind;
    // Mapping between segments and their corresponding axes's 
    // uncertainties.
    std::map<int, Eigen::Matrix3d> _axis_uncertainties;
    // Mapping between segments and the intermediate matrix-vector pairs
    // calculated while estimating the contour the corresponding segment.
    // This is a function of the form Ax=b.
    std::map<int, pair<Eigen::MatrixXd, Eigen::VectorXd> > _segment_contour_equ;
    // Mapping between segments and the derivative of the corresponding
    // contour w.r.t. 'x' the solution to Ax=b explained above.
    std::map<int, Eigen::MatrixXd> _segment_dcontour;
    // Mapping between the segments and the corresponding contour 
    // uncertainties.
    std::map<int, Eigen::MatrixXd> _contour_uncertainties;

    // ------------ POINTCLOUDS --------------------------------------------------- //
    // The original unfiltered point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_orig;
    // '_pc_orig' after filtered with voxel grid filter and radius filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_sphere;
    // Normals of the points in '_pc_sphere'.
    pcl::PointCloud<pcl::Normal>::Ptr _pc_sphere_normals;
    // '_pc_sphere' written in the corresponding segment frame.
    vector<Eigen::Vector3d> _pc_projections; 

    // ------------ NORMAL & UNCERTAINTY ESTIMATION ------------------------------- //
    // Some intermediate variables used in estimating surface normals.
    vector<vector<int  > > _nearest_neigh_inds;
    vector<vector<float> > _nearest_neigh_sq_dists;
    // Covariance and centroids for each point in '_pc_sphere' 
    vector<Eigen::Matrix3d> _normal_covs;
    vector<Eigen::Vector4d> _normal_centroids;
    // Eigenpairs of each '_normal_covs'
    vector<pair<Eigen::Vector3d, Eigen::Matrix3d> > _normal_eigenpairs;
    // Index of the smallest eigenvalue of the eigenpairs in '_normal_eigenpairs'
    vector<int> _normal_min_eigval_ind;
    // Uncertainties of each normals vector
    vector<Eigen::Matrix3d> _normal_uncertainties;
    // Uncertainties each point in '_pc_sphere' written in the sensor frame.
    vector<Eigen::Matrix3d> _point_uncertainties;

    // PCL's visualization toolbox
    pcl::visualization::PCLVisualizer::Ptr _viewer;

    // ---------------------------------------------------------------------------- //
    // This fucntion estimates surface normals for each point in '_pc_sphere' as well
    // as it estimates each normal's uncertainty.
    int _fit_normals();
    // This function initializes/refines the coordinate frame and the origin of the 
    // segment 'seg'. It also generates intermediate variables for later axis 
    // uncertainty estimation.
    int _init_triad(int seg);
    // This function iteratively filters for a segment, fits contour and eliminates
    // outliers until convergence.
    int _fit_segment(int seg);
    // This function assigns points satisfying the criterion given in '_params' to 'seg'.
    int _filter_segment(int seg);
    // This function eliminates outliers of segment 'seg' with either using
    // the 'normal' or 'contour' method.
    int _eliminate_outliers(int seg, const std::string &method);
    // This function transforms points of the segment 'seg' from the sensor
    // frame to the corresponding segment frame.
    int _project_pc(int seg);
    // This function fits a contour to the projected points of the segment 'seg'
    // using the method given in '_params'.
    int _fit_contour(int seg);
    // This function estimates axis and contour uncertainties of the segment 'seg'
    int _estimate_uncertainties(int seg);
  public:

    PC2Surfaces();
    PC2Surfaces(const PC2SurfacesParams &params);

    int push_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);

    inline int set_params(const PC2SurfacesParams &params){ 
      _params = params; 
      return 0;
    }

    inline int get_params(PC2SurfacesParams &params){ 
      params = _params; 
      return 0;
    }

    inline int get_segment_ids(vector<int> &ids, vector<bool> &outliers){
      ids = _segment_ids;
      outliers = _outliers;
      return _segment_ids.size();
    }

    inline int get_segments( 
        map<int, Eigen::Vector3d> &segment_origins, 
        map<int, Eigen::Matrix3d> &segment_triads, 
        map<int, Eigen::VectorXd> &segment_contours){
      segment_origins.clear();
      segment_triads.clear();
      segment_contours.clear();
      for(int i = 0 ; i < (int)_valid_segs.size() ; i++){
        int seg = _valid_segs[i];
        segment_origins[seg]  = _segment_origin_map[seg];
        segment_triads[seg]   = _segment_triad_map[seg];
        segment_contours[seg] = _segment_contour_map[seg];
      }
      return segment_triads.size();
    }

    inline int get_orig_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc){
      if(_pc_orig){
        pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*_pc_orig));
        return pc->points.size();
      } else {
        pc = NULL;
        return 0;
      }
    }

    inline int get_pc_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc){
      if(_pc_sphere){
        pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*_pc_sphere));
        return pc->points.size();
      } else {
        pc = NULL;
        return 0;
      }
    }

    inline int get_normals(pcl::PointCloud<pcl::Normal>::Ptr &normals){
      if(_pc_sphere_normals){
        normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>(*_pc_sphere_normals));
        return normals->points.size();
      } else {
        normals = NULL;
        return 0;
      }
    }

    inline int get_uncertainties(
        map<int, Eigen::Matrix3d> &axis_uncertainties, 
        map<int, Eigen::MatrixXd> &contour_uncertainties){
      axis_uncertainties =  _axis_uncertainties;
      contour_uncertainties =  _contour_uncertainties;
      return axis_uncertainties.size();
    }

    inline int get_point_uncertainties(vector<Eigen::Matrix3d> &covs){
      covs = _point_uncertainties;
      return covs.size();
    }

    inline int get_normal_uncertainties(vector<Eigen::Matrix3d> &covs){
      covs = _normal_uncertainties;
      return covs.size();
    }

    int get_projections(int seg, vector<Eigen::Vector3d> &proj, vector<bool> &outliers);
    int get_segment_pc(int seg, pcl::PointCloud<pcl::PointXYZ> &pc);

    int visualize_fit();
};



#endif
