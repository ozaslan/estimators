#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>


#include <visualization_msgs/MarkerArray.h>

#include "utils.hh"
#include "pc_to_surfaces.hh"

#define DEBUG_MSGS_ON 1
#define DEBUG_MSG_TEXT (string(__func__) + " " + to_string(__LINE__))

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher rviz_publ, pc_orig_publ, pc_sphere_publ, pc_sphere_normals_publ;
ros::Subscriber velodyne_subs;
sensor_msgs::PointCloud2 velodyne_msg;

pcl::PointCloud<pcl::PointXYZ>::Ptr local_map = NULL;

void process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
void velodyne_callback(const sensor_msgs::PointCloud2 &msg);
void publish_rviz_msgs();

std::map<int, Eigen::Vector3d> segment_origins;
std::map<int, Eigen::Matrix3d> segment_triads;
std::map<int, Eigen::VectorXd> segment_contours;

PC2Surfaces pc2surfs;

Eigen::Vector4d normals_color;
double normals_scale, normals_norm;
int normals_skip;
Eigen::Vector4d pc_sphere_color;
Eigen::Vector4d pc_orig_color;
bool plot_pc_orig;
bool plot_pc_sphere;
bool plot_normals;

bool debug_mode;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_icra2017_video");
  ros::NodeHandle n("~");

  process_inputs(n);	
  setup_messaging_interface(n);

  while(ros::ok()){
    ros::spinOnce();
  }

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

  n.param("visualization/pc_orig_color/r", pc_orig_color(0), 1.0);
  n.param("visualization/pc_orig_color/g", pc_orig_color(1), 1.0);
  n.param("visualization/pc_orig_color/b", pc_orig_color(2), 1.0);
  n.param("visualization/pc_orig_color/a", pc_orig_color(3), 1.0);
  n.param("visualization/pc_sphere_color/r", pc_sphere_color(0), 1.0);
  n.param("visualization/pc_sphere_color/g", pc_sphere_color(1), 1.0);
  n.param("visualization/pc_sphere_color/b", pc_sphere_color(2), 1.0);
  n.param("visualization/pc_sphere_color/a", pc_sphere_color(3), 1.0);
  n.param("visualization/normals_color/r", normals_color(0), 1.0);
  n.param("visualization/normals_color/g", normals_color(1), 1.0);
  n.param("visualization/normals_color/b", normals_color(2), 1.0);
  n.param("visualization/normals_color/a", normals_color(3), 1.0);
  n.param("visualization/normals_scale", normals_scale, 0.01);
  n.param("visualization/normals_norm", normals_norm, 1.0);
  n.param("visualization/normals_skip", normals_skip, 10);
  n.param("visualization/plot_pc_orig", plot_pc_orig, true);
  n.param("visualization/plot_pc_sphere", plot_pc_sphere, true);
  n.param("visualization/plot_normals", plot_normals, true);

  n.param("debug_mode", debug_mode, false);



  ROS_INFO(" ---------- TEST_ICRA2017 NODE ------------");
  ROS_INFO("[debug_mode] : [%s]", debug_mode ? "TRUE" : "FALSE");
  params.print();
  ROS_INFO(" ------------------------------------------");
}

int setup_messaging_interface(ros::NodeHandle &n)
{
  if(debug_mode)	{
    ROS_INFO("TEST ICRA2017_VIDEO NODE : setting up messaging interface.");
    ROS_INFO(" --- Listening  : ~velodyne_points");
  }

  velodyne_subs	 = n.subscribe("velodyne_points", 10, velodyne_callback, ros::TransportHints().tcpNoDelay());
  rviz_publ      = n.advertise<visualization_msgs::MarkerArray>("markers", 10);
  pc_orig_publ   = n.advertise<PointCloud> ("pc_orig", 1);
  pc_sphere_publ = n.advertise<PointCloud> ("pc_sphere", 1);
  pc_sphere_normals_publ = n.advertise<PointCloud> ("pc_sphere_normals", 1);
  return 0;
}

void velodyne_callback(const sensor_msgs::PointCloud2 &msg)
{
  if(debug_mode)
    ROS_INFO("TEST ICRA2017_VIDEO NODE : Got Velodyne message!");

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc =
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg, *pc);

  pc2surfs.push_pc(pc);
  publish_rviz_msgs();
  // pc2surfs.visualize_fit();

  // pc2surfs.get_segments(segment_origins, segment_triads, segment_contours);
}

void publish_rviz_msgs(){
  static int seq = 0;
  DEBUG_MSG(DEBUG_MSG_TEXT);
  // Declerations and initializations
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();
  marker.header.seq = seq++;
  marker.ns = "velodyne";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.lifetime = ros::Duration(0);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;


  // ---------------------------------------------------------- //
  vector<int>  ids;
  vector<bool> outliers;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_orig, pc_sphere;
  pcl::PointCloud<pcl::Normal>::Ptr pc_sphere_normals;
  // ---------------------------------------------------------- //
  //
  // pc2surfs.get_segment_ids(ids, outliers);

  // Publish _pc_orig
  if(plot_pc_orig && pc2surfs.get_orig_pc(pc_orig)){
    pc_orig->header.frame_id = "velodyne";
    pc_orig->height = 1;
    pc_orig->width = pc_orig->points.size();
    pc_orig->header.stamp = ros::Time::now().toSec();
    pc_orig_publ.publish(*pc_orig);
  }
  // Publish _pc_sphere 
  if((plot_pc_sphere || plot_normals) && pc2surfs.get_pc_sphere(pc_sphere)){
    if(plot_pc_sphere){
      pc_sphere->header.frame_id = "velodyne";
      pc_sphere->height = 1;
      pc_sphere->width = pc_sphere->points.size();
      pc_sphere->header.stamp = ros::Time::now().toSec();
      pc_sphere_publ.publish(*pc_sphere);
    }
  }
  // Plot normals
  if(plot_normals && pc2surfs.get_normals(pc_sphere_normals)){
    int num_pts = pc_sphere_normals->points.size();
    marker.points.clear();
    marker.points.reserve(num_pts * 2);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = normals_scale;
    marker.scale.y = normals_scale;
    marker.scale.z = normals_scale;
    int skip = normals_skip;

    marker.color.r = normals_color(0);
    marker.color.g = normals_color(1);
    marker.color.b = normals_color(2);
    marker.color.a = normals_color(3);

    for(int i = 0 ; i < num_pts ; i++){
      if(i % skip != 0 )
        continue;
      pcl::PointXYZ pt = pc_sphere->points[i];
      pcl::Normal normal = pc_sphere_normals->points[i];
      geometry_msgs::Point pt1, pt2;
      pt1.x = pt.x; pt1.y = pt.y; pt1.z = pt.z;
      pt2.x = pt.x + normals_norm * normal.normal_x; 
      pt2.y = pt.y + normals_norm * normal.normal_y; 
      pt2.z = pt.z + normals_norm * normal.normal_z;
      marker.points.push_back(pt1);
      marker.points.push_back(pt2);
    }
    markers.markers.push_back(marker);
    /*
       pc_sphere_normals->header.frame_id = "velodyne";
       pc_sphere_normals->height = 1;
       pc_sphere_normals->width = pc_sphere_normals->points.size();
       pc_sphere_normals->header.stamp = ros::Time::now().toSec();
       pc_sphere_normals_publ.publish(*pc_sphere_normals);
       */
  }
  // Initial axis estimate
  std::map<int, Eigen::Vector3d> segment_origins;
  std::map<int, Eigen::Matrix3d> segment_triads;
  std::map<int, Eigen::VectorXd> segment_contours;
  pc2surfs.get_segments(segment_origins, segment_triads, segment_contours);



  // Iterate through segments
  // - Project onto prev triad
  // - Draw plane
  // - Measure distances to plane
  // - Make a new segment, show pc
  // - Fit a new triad, origin
  // - Refine
  // - Go to the first step until no segments left

  //

  //

  //
  rviz_publ.publish(markers);
}

