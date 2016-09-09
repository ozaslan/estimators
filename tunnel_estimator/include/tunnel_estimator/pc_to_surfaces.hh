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
    PC2SurfacesParams(){
      max_inner_iter = 33;
      max_outer_iter = 33;
      outlier_id = -99999;
      unclassified_id = 99999;
      term_perc_crit = 0.01;
      normal_dev_thres1 = 5; // degrees
      normal_dev_thres2 = 5; 
      contour_fit_thres = 0.05; // ratio to radius
      sphere_r = 7.5; // meters
      segment_len = 1; // meters
      normal_search_radius = 0.25;
      contour_type = "circle";
    }

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
    string contour_type;
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
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> _ne;
    vector<Eigen::Vector3d> _pc_projections; // y-z components are plane coordinates

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
    int fit_surfaces();
};



#endif
