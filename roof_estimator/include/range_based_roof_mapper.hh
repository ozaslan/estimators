#ifndef __RANGE_BASED_ROOF_MAPPER_HH__
#define __RANGE_BASED_ROOF_MAPPER_HH__

#include <cmath>
#include <vector>
#include <limits>
#include <numeric>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <calib_params.hh>

#include <octomap/OcTree.h>

#include <utils.hh>

/*

-- This class gets the scan data / rgbd data together with 
   the pose at which the data is obtained. Then, it integrates 
   the sensor information to the octomap::OcTree
   structure in a probabilistic way (see the paper). If the map is empty, 
   (initializtion phase) the laser data is extruded in the 
   sensor +-z directions upto a user defined lenght. For multiple 
   laser scanner scenarios this operation is carried for each 
   laser. In the case of rgbd data, this class does not apply 
   any special operation. With the above method, we assume a 
   dense environment map that will provide sufficient information 
   for the initial short term localization. Over time, with the new sensor 
   data, this map is updated to reflect the actual world structure.
*/

class RangeBasedRoofMapper{
private:
  octomap::OcTree _octree;
  octomap::Pointcloud _cloud;
  // Resolution of the octree.
  double _res;
public:
  // This constructor initializes the private OcTree instance
  // with the given resolution.
  RangeBasedRoofMapper(double res = 0.05);
  // This function returns a constant reference to the private octree
  // class. It can be used for visualization purposes.
  const octomap::OcTree& get_map();
  // This function integrates the laser scan into the octree where 'pose' is 
  // the sensor pose. If 'extrude' is set, the scan is extruded 
  // in the +-z directions of the scanner. This generates a 3D approximation 
  // of the map to help localization until further data is provided. Thus it 
  // is not recommended to set 'extrude=true' except the initialization phase.
  // Multiple extrusions can be done in case of multiple laser scanners.
  // If 'mask.size() = 0' all the scan data is used. If 'mask.size() = scan.ranges.size()'
  // then all the data with 'mask[i] = false' is discarded. Different 'mask'
  // and 'ranges' sizes causes an exception.
  bool register_scan (const Eigen::Matrix4d &pose, const sensor_msgs::LaserScan &scan, const vector<char> &mask, char cluster_id, double extrude = 0);
  // This function integrates the point cloud into the octree where 'pose'
  // is the pose of the sensor. If 'mask.size() = 0' all the data is used. 
  // If 'mask.size() = cloud.points.size()' then all the data with 'mask[i] = false' 
  // is discarded. Different 'mask' and 'points' sizes causes an exception.
  bool register_cloud(const Eigen::Matrix4d &pose, const sensor_msgs::PointCloud2 &cloud, const vector<char> &mask, char cluster_id);
  // This function integrates a single scan/beam into the octree where 'from' and
  // 'to' defines the ray. 'cone_angle' encodes the behaviour of the sensor.
  // In the case of a sonar sensor, this should be set to a non-zero value for
  // more realistic maps. Laser scanners usually have small beam cones and can be 
  // left as '0' which results in update of only a single voxel.
  bool register_ray  (const Eigen::Vector3d &from, const Eigen::Vector3d &to, double cone_angle = 0);
  // This function returns the bounding box of the octree.
  bool get_BBX(Eigen::Vector3d &min, Eigen::Vector3d &max);
  // This function returns the resolution of the octree.
  inline double get_resolution(){return _res;}
  // This function returns the size of the octree as occupied in the memory.
  // The size is in kilobytes.
  double get_memory_usage();
  // This function resets the octomap.
  bool reset();
  const octomap::Pointcloud&  get_last_registration(){ return _cloud;}
};

#endif
