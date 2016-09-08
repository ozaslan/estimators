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

class PC2Surfaces{

  private:
    int _max_inner_iter = 33;
    int _max_outer_iter = 33;
    int _outlier_id = -99999;
    int _unclassified_id = 99999;
    double _term_perc_crit = 0.01;
    double _normal_dev_thres1 = 5; // degrees
    double _normal_dev_thres2 = 5; 
    double _cylinder_fit_thres = 0.05; // ratio to radius
    double _sphere_r = 7.5; // meters
    double _segment_len = 1; // meters
    double _normal_search_radius = 0.25;
    
    vector<int> _segment_ids;
    std::map<int, Eigen::Matrix3d> _segment_triad_map;
    std::map<int, Eigen::Vector3d> _segment_origin_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_orig;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_sphere;
    pcl::PointCloud<pcl::Normal>::Ptr _pc_sphere_normals;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> _ne;

    bool _fit_segment(int seg);
  public:
    PC2Surfaces();
    
    int push_pc(const pcl::PointCloud<pcl::PointXYZ>::ptr &pc);
    int fit_surfaces();
};


#endif
