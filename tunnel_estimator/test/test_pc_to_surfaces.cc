#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "utils.hh"
#include "pc_to_surfaces.hh"

double x = 0;

ros::Subscriber velodyne_subs;
sensor_msgs::PointCloud2 velodyne_msg;

pcl::PointCloud<pcl::PointXYZ>::Ptr local_map = NULL;

void process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
void velodyne_callback(const sensor_msgs::PointCloud2 &msg);

std::map<int, Eigen::Vector3d> segment_origins;
std::map<int, Eigen::Matrix3d> segment_triads;
std::map<int, Eigen::VectorXd> segment_contours;

PC2Surfaces pc2surfs;
bool debug_mode;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_pc_to_surfaces");
  ros::NodeHandle n("~");

  process_inputs(n);	
  setup_messaging_interface(n);

  ros::spin();

  return 0;
}

void process_inputs(const ros::NodeHandle &n)
{
  PC2SurfacesParams params;
  n.param("pc2surfaces/max_inner_iter", params.max_inner_iter, 33);
  n.param("pc2surfaces/max_outer_iter", params.max_outer_iter, 33);
  n.param("pc2surfaces/term_perc_crit", params.term_perc_crit, 0.05);
  n.param("pc2surfaces/normal_dev_thres1", params.normal_dev_thres1, 5.0);
  n.param("pc2surfaces/normal_dev_thres2", params.normal_dev_thres2, 5.0);
  n.param("pc2surfaces/contour_fit_thres", params.contour_fit_thres, 0.05);
  n.param("pc2surfaces/sphere_r", params.sphere_r, 5.5);
  n.param("pc2surfaces/normal_search_radius", params.normal_search_radius, 0.25);
  n.param("pc2surfaces/contour_type", params.contour_type, std::string("circle"));
  n.param("pc2surfaces/segment_len", params.segment_len, 1.0);
  n.param("pc2surfaces/num_segments", params.num_segments, 3);
  n.param("pc2surfaces/voxel_leaf_size", params.voxel_leaf_size, 0.03);
  n.param("pc2surfaces/curvature_thres", params.curvature_thres, 0.10);
  n.param("pc2surfaces/var_r", params.var_r, 0.001);
  n.param("pc2surfaces/var_azimutal", params.var_azimutal, 0.001);
  n.param("pc2surfaces/var_elevation", params.var_elevation, 0.001);

  n.param("debug_mode", debug_mode, false);



  ROS_INFO(" ---------- TEST_PC2SURFACES NODE ------------");
  ROS_INFO("[debug_mode] : [%s]", debug_mode ? "TRUE" : "FALSE");
  params.print();
  ROS_INFO(" ------------------------------------------");
}

int setup_messaging_interface(ros::NodeHandle &n)
{
  if(debug_mode)	{
    ROS_INFO("VELODYNE ODOM NODE : setting up messaging interface.");
    ROS_INFO(" --- Listening  : ~velodyne_points");
    ROS_INFO(" --- Publishing : ~map");
    ROS_INFO(" --- Publishing : ~odom");
    ROS_INFO(" --- Publishing : ~aligned_pc");
    ROS_INFO(" --- Publishing : ~keyframe_pc");
  }

  velodyne_subs	= n.subscribe("velodyne_points", 10, velodyne_callback, ros::TransportHints().tcpNoDelay());

  return 0;
}

void velodyne_callback(const sensor_msgs::PointCloud2 &msg)
{
  if(debug_mode)
    ROS_INFO("TEST PC2SURFACES NODE : Got Velodyne message!");

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc =
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg, *pc);

  pc2surfs.push_pc(pc);
  pc2surfs.visualize_fit();

  pc2surfs.get_segments(segment_origins, segment_triads, segment_contours);

}

