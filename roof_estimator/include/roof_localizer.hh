// !!! Definition : An estimator is a process which takes
// sensor inputs and some parameters as input, and outputs
// a full/partial pose estimate.
// Input-1  : sensor data
// Input-2  : parameters
// Output-1 : pose estimate

#ifndef __ROOF_LOCALIZER_HH__
#define __ROOF_LOCALIZER_HH__

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h> // If you forget this, publish will give a serialization error.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/registration/icp.h>

#include <calib_params.hh>
#include "range_based_roof_localizer.hh"
#include "vision_based_roof_localizer.hh"

class RoofLocalizer{
  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _map;
    RangeBasedRoofLocalizer _rbrl;
    VisionBasedRoofLocalizer _vbrl;
    vector<cv::Mat> _frames;
    vector<CameraCalibParams>		_cam_params;
    double _x_disp, _x_pos;

    vector<bool> _camera_data_available;
    vector<bool> _lidar_data_available;
    vector<std::pair<double, double> > _lidar_stamps, _camera_stamps;
    vector<string> _lidar_ids, _camera_ids;

  public:
    // The following functions process raw data and register to the proper localizer lower-level estimators.
    // After registration estimate(...) function has to be called explicitely. Before registering new
    // sensor data, reset(...) has to be called in order to forget previous sensor information.
    bool push_lidar_data (const sensor_msgs::LaserScan &scan  , const LidarCalibParams  &params, bool clean_start = false);
    bool push_rgbd_data  (const sensor_msgs::PointCloud2 &rdgb, const RGBDCalibParams   &params, bool clean_start = false);
    bool push_camera_data(const sensor_msgs::Image &frame	  , const CameraCalibParams &params);
    // This function calls RangeBasedTunnelLocalizer's and VisionBasedTunnelLocalizer's
    // estimators. Depending on the choice of parameters (### to beimplemented) either
    // a ranges-major, vision-aided estimator or a sensor-fusion-based estimator scheme
    // is executed. Also the user can set the free and non-free dimensions to constrain 
    // the directions of pose update/estimate (### to be implemented). After calling this, 
    // reset(...) has to be called explicitely to flush the memory. Results of the esti-
    // mation can be retreived through get_pose(...) and get_covariance(...) functions.
    // ### Think for more alternative properties of this function.
    bool estimate_pose(const Eigen::Matrix4d &init_pose, const octomap::OcTree &octomap);
    // This function calls the reset(...) functions of RangeBasedTunnelLocalizer and
    // VisionBasedTunnelLocalizer. Those functions reset the private variables including 
    // the cached/collective sensor data and some internal flags. For this reason, 
    // after a call to estimate_pose(...), this has to be called before registering new 
    // sensor data for the next estimation step. 
    bool reset();
    // This function updates the internal map pointer. After each update set_map(...)
    // functions of RangeBasedTunnelLocalizer and VisionBasedTunnelLocalizer are called.
    // Due to possible heavy computations (such as kd-tree generation for RangeBasedTunnelLocalizer)
    // this function should not be called frequently.
    bool set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &grid_map);
    // The below two functions return the pose and covariance estimates.
    bool get_pose(Eigen::Matrix4d &pose);
    bool get_covariance(Eigen::Matrix6d &cov);

    bool get_registered_lidar_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud );

    bool get_range_map_correspondences(vector<pcl::PointXYZ> &range_pts, vector<pcl::PointXYZ> &map_pts);

    bool plot_tracked_features(cv::Mat &image, bool plot_flow = true, bool plot_feats = true);

    bool get_back_projected_features(vector<pcl::PointXYZ> &tails, vector<pcl::PointXYZ> &tips);

    bool register_camera(const CameraCalibParams &params);
};


#endif


