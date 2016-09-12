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

// ### #include <pcl/registration/gicp.h>
// #include <pcl/filters/approximate_voxel_grid.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/passthrough.h>
//  
//  #include <ros/ros.h>
//   
//   #include <sensor_msgs/PointCloud2.h>
//

#include <Eigen/Eigenvalues> 


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
  };

class PC2Surfaces{
  private:
    PC2SurfacesParams _params;

    vector<int> _segment_ids;
    std::map<int, Eigen::Matrix3d> _segment_triad_map;
    std::map<int, Eigen::Vector3d> _segment_origin_map;
    std::map<int, Eigen::VectorXd> _segment_contour_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_orig;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_sphere;
    pcl::PointCloud<pcl::Normal>::Ptr _pc_sphere_normals;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> _ne;
    vector<Eigen::Vector3d> _pc_projections; // y-z components are plane coordinates
    vector<bool> _outliers;

    vector<vector<int  > > _nearest_neigh_inds;
    vector<vector<float> > _nearest_neigh_sq_dists;
    vector<Eigen::Matrix3d> _normal_covs;
    vector<Eigen::Vector4d> _normal_centroids;
    vector<pair<Eigen::Vector3d, Eigen::Matrix3d> > _normal_eigenpairs;
    vector<int> _normal_min_eigval_ind;
    vector<Eigen::Matrix3d> _normal_uncertainties;
    vector<Eigen::Matrix3d> _point_uncertainties;


    pcl::visualization::PCLVisualizer::Ptr _viewer;

    int _max_seg_ind;

    // This function fits surface normals to '_pc_sphere' and returns the
    // number of normal vectors
    int _fit_normals();
    // 
    int _init_triad(int seg);
    int _fit_segment(int seg);
    int _filter_segment(int seg);
    int _eliminate_outliers(int seg, const std::string &method);
    int _project_pc(int seg);
    int _fit_contour(int seg);
  public:
    PC2Surfaces();
    PC2Surfaces(const PC2SurfacesParams &params);

    int push_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);

    int set_params(const PC2SurfacesParams &params){ _params = params; return 0;}

    int visualize_fit();
};



#endif
