/*
   This code simply generates a point cloud for a rectangular shaped
   corridor with closed ends. It uses PCL to internally represent the
   structure and finally saves it to a '.pcd' file to be later loaded
   from other nodes.

*/

/*

   - load pcd file
   - find the bounding box
   - sample points until inside the tunnel
   - sample the next point, iteratively correct the position by keeping r = r0
   - sample points at the tip of the vector while inside
   - sample points at the tip of the minus the vector while inside
   - write on a file with the same name, different extension.
   - make a simulation
   */

#include<cmath>
#include<string>
#include<cstdlib>
#include<iostream>
#include<algorithm>

#include<Eigen/Dense> 

#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/file_io.h>

using namespace std;

int main(int argc, char* argv[]){
  string cloud_path;

  // Get the cloud_path
  if(argc != 2){
    cout << "Usage: extract_tunnel_center_line <input.pcd>" << endl;
    exit(-1);
  }
  cloud_path = string(argv[2]);

  // Load the map from the given '.pcd' file path.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
  cloud_ptr = cloud.makeShared();
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (cloud_path.c_str(), *cloud_ptr) == -1){                                                                                             ROS_ERROR ("Couldn't read file map file from the path : %s", map_path.c_str());
  }  

  // Generate the corresponding octree.
  octree= new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.05);                                                                                            
  octree->setInputCloud(cloud_ptr);
  octree->addPointsFromInputCloud();

  // Find the bounding box
  pcl::pointXYZ min_corner, max_corner;
  min_corner = cloud_ptr->points[0];
  max_corner = cloud_ptr->points[0];
  for(int i = 1 ; i < cloud_ptr->points.size() ; i++){
    pcl::Point pt = cloud_ptr->points[i];
    if(pt.x < min_corner.x || pt.y < min_corner.y || pt.z < min_corner.z)
      min_corner = pt;
    if(pt.x > max_corner.x || pt.y > max_corner.y || pt.z > max_corner.z)
      max_corner = pt;
  }

  // The distance between consecutive center line point
  double R = 1; 

  // Vectors of point constituting the center line
  vector<Eigen::Vector3f> center_line;
  // Sample points from inside the tunnel
  // Initialize the center line with the seed and a second point
  while(true){
    // Sample a point inside the bouding box.
    Eigen::Vector3f pt;
    pt(0) = min_point.x + (max_point.x - min_point.x) * rand() / (double)RAND_MAX;
    pt(1) = min_point.y + (max_point.y - min_point.y) * rand() / (double)RAND_MAX;
    pt(2) = min_point.z + (max_point.z - min_point.z) * rand() / (double)RAND_MAX;

    if(is_inside(octree, pt)){
      // Direction vector pointing towards 'forward'
      Eigen::Vector3f dir = pt - center_line.back();
      align_point(octree, pt, dir);
      center_line.push_back(pt);
      pt.x = center_line[0].x + dir.x * R;
      pt.y = center_line[0].y + dir.y * R;
      pt.z = center_line[0].z + dir.z * R;
      break;
    }    
  }

  // Continue adding points until point lies outside the map
  while(true){
    Eigen::Vector3f dir, pt;
    dir = center_line[center_line.size() - 1] - center_line[center_line.size() - 2];
    pt = center_line.back() + dir / dir.norm() * R;
    if(is_inside(pt, octree)){
      align_point(octree, pt, dir);
      center_line.push_back(pt);
    } else
      break;
  }

  return 0;
}

double intersect_sphere(const Eigen::Vector3f &pt, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, vector<Eigen::Vector3f> &intercepts){
  // Generate the rays pointing to 'all' directions around the pivot    
  // Intersect of all them with the octree
  double PI = 3.14;
  double R = 1;
  double az, pol;
  intercepts.clear();
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::AlignedPointTVector voxel_centers;
  int num_rays = 0;
  for(az = 0 ; az < PI ; az += 0.087){
    for(pol = 0 ; pol < 2 * PI ; po += 0.087){
      Eigen::Vector3f ray;
      ray(0) = sin(az) * cos(pol);
      ray(1) = sin(az) * sin(pol);
      ray(2) = cos(az);
      _octree->getIntersectedVoxelCenters(p, ray, voxel_centers , 1);
      num_rays++;
      if(voxel_centers.size() != 0)
        intercepts.push_back(Eigen::Vector3f(voxel_centers[0].x,
              voxel_centers[0].y,
              voxel_centers[0].z));
    }
  }
  return (double)intercepts.size() / num_rays;
}

bool is_inside(const Eigen::Vector3f &pt, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree){
  // intersect_sphere with 'pt' being the pivot.
  // Count the total number of intersecting one
  // If more than half is not intersecting, then the point is outside
  vector<Eigen::Vector3f> intercepts;
  return intersect_sphere(pt, octree, intercepts) > 0.5;
}

bool align_point(Eigen::Vector3f &pt, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, const Eigen::Vector3f &dir){
  Eigen::Vector3f z_axis(0, 0, 1);
  Eigen::Vector3f y_axis = dir.cross(z_axis);
  y_axis /= y_axis.norm();

  vector<Eigen::Vector3f> intercepts;
  intersect_sphere(pt, octree, intercepts);

}
