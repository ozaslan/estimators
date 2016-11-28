#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>


#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "utils.hh"
#include "pc_to_surfaces.hh"

#define DEBUG_MSGS_ON 1
#define DEBUG_MSG_TEXT (string(__func__) + " " + to_string(__LINE__))

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> ColoredPointCloud;

utils::colors::Colors colors;

ros::Publisher rviz_publ, 
  pc_orig_publ, 
  pc_sphere_publ, 
  pc_sphere_normals_publ,
  normals_publ,
  triads_publ,
  planes_publ,
  segments_publ,
  cylinders_publ,
  sensor_frame_publ,
  odom_publ;
ros::Subscriber velodyne_subs, imu_subs;
sensor_msgs::PointCloud2 velodyne_msg;
nav_msgs::Odometry odom_msg;
sensor_msgs::Imu imu_msg;

pcl::PointCloud<pcl::PointXYZ>::Ptr local_map = NULL;


void process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
void velodyne_callback(const sensor_msgs::PointCloud2 &msg);
void imu_callback(const sensor_msgs::Imu &msg);
void publish_rviz_msgs();
void publish_odom_msg();

std::map<int, Eigen::Vector3d> segment_origins;
std::map<int, Eigen::Matrix3d> segment_triads;
std::map<int, Eigen::VectorXd> segment_contours;

PC2Surfaces pc2surfs;

Eigen::Vector4d normals_color;
double normals_scale, normals_norm;
double triads_scale, triads_norm;
double planes_scale;
int normals_skip;
Eigen::Vector4d pc_sphere_color;
Eigen::Vector4d pc_orig_color;
double cylinders_alpha;
double cylinders_line_scale;
int cylinders_resolution;
bool plot_pc_orig;
bool plot_pc_sphere;
bool plot_normals;
bool plot_triads;
bool plot_planes;
bool plot_cylinders;
bool plot_segments;
bool plot_sensor_frame;
bool enable_pcl_viewer;

bool debug_mode;
bool imu_available = false;

PC2SurfacesParams params;

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
  n.param("pc2surfaces/max_segment_dist", params.max_segment_dist, 1.2);
  n.param("pc2surfaces/max_segment_rot", params.max_segment_rot, DEG2RAD(10));

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
  n.param("visualization/cylinders_alpha", cylinders_alpha, 0.7);
  n.param("visualization/normals_scale", normals_scale, 0.01);
  n.param("visualization/triads_scale", triads_scale, 0.01);
  n.param("visualization/planes_scale", planes_scale, 1.0);
  n.param("visualization/cylinders_line_scale", cylinders_line_scale, 0.01);
  n.param("visualization/cylinders_resolution", cylinders_resolution, 30);
  n.param("visualization/normals_norm", normals_norm, 1.0);
  n.param("visualization/triads_norm", triads_norm, 1.0);
  n.param("visualization/normals_skip", normals_skip, 10);
  n.param("visualization/plot_pc_orig", plot_pc_orig, true);
  n.param("visualization/plot_pc_sphere", plot_pc_sphere, true);
  n.param("visualization/plot_normals", plot_normals, true);
  n.param("visualization/plot_triads", plot_triads, true);
  n.param("visualization/plot_planes", plot_planes, true);
  n.param("visualization/plot_cylinders", plot_cylinders, true);
  n.param("visualization/plot_segments", plot_segments, true);
  n.param("visualization/plot_sensor_frame", plot_sensor_frame, true);
  n.param("visualization/enable_pcl_viewer", enable_pcl_viewer, false);

  n.param("debug_mode", debug_mode, false);



  ROS_INFO(" ---------- TEST_ICRA2017 NODE ------------");
  ROS_INFO("[debug_mode] : [%s]", debug_mode ? "TRUE" : "FALSE");
  params.print();
  ROS_INFO(" ------------------------------------------");

  pc2surfs.set_params(params);
}

int setup_messaging_interface(ros::NodeHandle &n)
{
  if(debug_mode)	{
    ROS_INFO("TEST ICRA2017_VIDEO NODE : setting up messaging interface.");
    ROS_INFO(" --- Listening  : ~velodyne_points");
  }

  velodyne_subs	 = n.subscribe("velodyne_points", 10, velodyne_callback, ros::TransportHints().tcpNoDelay());
  imu_subs	 = n.subscribe("imu", 10, imu_callback, ros::TransportHints().tcpNoDelay());
  rviz_publ      = n.advertise<visualization_msgs::Marker>("delete_markers", 10);
  cylinders_publ = n.advertise<visualization_msgs::MarkerArray>("cylinders", 10);
  normals_publ   = n.advertise<visualization_msgs::Marker>("normals", 10);
  triads_publ    = n.advertise<visualization_msgs::Marker>("triads", 10);
  sensor_frame_publ = n.advertise<visualization_msgs::Marker>("sensor_frame", 10);
  planes_publ    = n.advertise<visualization_msgs::MarkerArray>("planes", 10);
  pc_orig_publ   = n.advertise<PointCloud> ("pc_orig", 1);
  pc_sphere_publ = n.advertise<PointCloud> ("pc_sphere", 1);
  segments_publ  = n.advertise<ColoredPointCloud> ("pc_segments", 1);
  odom_publ      = n.advertise<nav_msgs::Odometry> ("odom", 1);
  pc_sphere_normals_publ = n.advertise<PointCloud> ("pc_sphere_normals", 1);

  return 0;
}

void imu_callback(const sensor_msgs::Imu &msg){
  imu_msg = msg;
  imu_available = true;
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
  publish_odom_msg();
  if(enable_pcl_viewer)
    pc2surfs.visualize_fit();

  // pc2surfs.get_segments(segment_origins, segment_triads, segment_contours);
}

void publish_rviz_msgs(){
  static int seq = 0;
  static int marker_id = 0;
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
  marker.lifetime = ros::Duration(0.3);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;


  marker.action = visualization_msgs::Marker::DELETE;
  for(int i = 0 ; i <= marker_id * 100 ; i++){
    marker.header.seq++;
    marker.id = i;
    rviz_publ.publish(marker);
  }
  marker.id = 0;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;

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
    pc_orig->header.seq = seq++;
    pc_orig->height = 1;
    pc_orig->width = pc_orig->points.size();
    pc_orig->header.stamp = ros::Time::now().toNSec();
    pc_orig_publ.publish(*pc_orig);
  }
  // Publish _pc_sphere 
  if((plot_pc_sphere || plot_normals) && pc2surfs.get_pc_sphere(pc_sphere)){
    if(plot_pc_sphere){
      pc_sphere->header.frame_id = "velodyne";
      pc_sphere->header.seq = seq++;
      pc_sphere->height = 1;
      pc_sphere->width = pc_sphere->points.size();
      pc_sphere->header.stamp = ros::Time::now().toNSec();
      pc_sphere_publ.publish(*pc_sphere);
    }
  }
  // Plot normals
  if(plot_normals && pc2surfs.get_normals(pc_sphere_normals)){
    int num_pts = pc_sphere_normals->points.size();
    marker.points.clear();
    marker.colors.clear();
    marker.colors.reserve(num_pts * 2);
    marker.points.reserve(num_pts * 2);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.header.seq++;
    marker.id++;
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
    normals_publ.publish(marker);
  }

  std::map<int, Eigen::Vector3d> segment_origins;
  std::map<int, Eigen::Matrix3d> segment_triads;
  std::map<int, Eigen::VectorXd> segment_contours;
  pc2surfs.get_segments(segment_origins, segment_triads, segment_contours);

  if(plot_triads){
    int num_segments = segment_origins.size();
    marker.points.clear();
    marker.colors.clear();
    marker.points.reserve(num_segments);
    marker.colors.reserve(num_segments);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.header.seq++;
    marker.id++;
    marker.scale.x = triads_scale;
    marker.scale.y = triads_scale;
    marker.scale.z = triads_scale;

    std_msgs::ColorRGBA red, green, blue;
    red.r = 1; 
    green.g = 1;
    blue.b = 1;
    red.a = green.a = blue.a = 1;

    Eigen::Vector3d origin;
    Eigen::Matrix3d triad;
    geometry_msgs::Point pt1, pt2;

    bool goon = true;
    for(int seg = 0 ; goon ; seg++){
      goon = false;
      auto it = segment_origins.find(seg);
      if(it != segment_origins.end()){
        origin = it->second;
        triad  = segment_triads[seg];
        if(!triad.allFinite() || fabs(triad.determinant() - 1) > 0.01 || !origin.allFinite()){
          goon = true;
          continue;
        }
        triad *= triads_norm;
        pt1.x = origin(0); pt1.y = origin(1); pt1.z = origin(2);
        pt2.x = pt1.x + triad(0, 0); pt2.y = pt1.y + triad(1, 0); pt2.z = pt1.z + triad(2, 0);
        marker.points.push_back(pt1); marker.points.push_back(pt2);
        marker.colors.push_back(red);
        marker.colors.push_back(red);
        pt2.x = pt1.x + triad(0, 1); pt2.y = pt1.y + triad(1, 1); pt2.z = pt1.z + triad(2, 1);
        marker.points.push_back(pt1); marker.points.push_back(pt2);
        marker.colors.push_back(green);
        marker.colors.push_back(green);
        pt2.x = pt1.x + triad(0, 2); pt2.y = pt1.y + triad(1, 2); pt2.z = pt1.z + triad(2, 2);
        marker.points.push_back(pt1); marker.points.push_back(pt2);
        marker.colors.push_back(blue);
        marker.colors.push_back(blue);
        goon = true;
      }

      it = segment_origins.find(-seg);
      if(seg != 0 && it != segment_origins.end()){
        origin = it->second;
        triad  = segment_triads[seg];
        if(!triad.allFinite() || fabs(triad.determinant() - 1) > 0.01 || !origin.allFinite()){
          goon = true;
          continue;
        }
        triad *= triads_norm;
        pt1.x = origin(0); pt1.y = origin(1); pt1.z = origin(2);
        pt2.x = pt1.x + triad(0, 0); pt2.y = pt1.y + triad(1, 0); pt2.z = pt1.z + triad(2, 0);
        marker.points.push_back(pt1); marker.points.push_back(pt2);
        marker.colors.push_back(red);
        marker.colors.push_back(red);
        pt2.x = pt1.x + triad(0, 1); pt2.y = pt1.y + triad(1, 1); pt2.z = pt1.z + triad(2, 1);
        marker.points.push_back(pt1); marker.points.push_back(pt2);
        marker.colors.push_back(green);
        marker.colors.push_back(green);
        pt2.x = pt1.x + triad(0, 2); pt2.y = pt1.y + triad(1, 2); pt2.z = pt1.z + triad(2, 2);
        marker.points.push_back(pt1); marker.points.push_back(pt2);
        marker.colors.push_back(blue);
        marker.colors.push_back(blue);
        goon = true;
      }
    }
    triads_publ.publish(marker);
  }

  if(plot_planes){
    markers.markers.clear();
    marker.colors.clear();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.01;
    marker.scale.y = planes_scale;
    marker.scale.z = planes_scale;

    Eigen::Vector3d origin;
    Eigen::Matrix3d triad;
    geometry_msgs::Point pt;
    pt.x = pt.y = pt.z = 0;
    marker.points.clear();
    marker.points.push_back(pt);
    marker.colors.clear();
    marker.color.a = 0.3;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;

    bool goon = true;
    for(int seg = 0 ; goon ; seg++){
      goon = false;
      auto it = segment_origins.find(seg);
      if(it != segment_origins.end()){
        origin = it->second;
        triad = segment_triads[seg];
        if(!triad.allFinite() || fabs(triad.determinant() - 1) > 0.01 || !origin.allFinite()){
          goon = true;
          continue;
        }
        marker.header.seq++;
        marker.id++;
        marker.pose.orientation = utils::trans::quat2quat(utils::trans::dcm2quat(triad));
        marker.pose.position.x = origin(0);
        marker.pose.position.y = origin(1);
        marker.pose.position.z = origin(2);
        // marker.colors.push_back(red);
        markers.markers.push_back(marker);
        goon = true;
      }

      it = segment_origins.find(-seg);
      if(seg != 0 && it != segment_origins.end()){
        origin = it->second;
        triad = segment_triads[seg];
        if(!triad.allFinite() || fabs(triad.determinant() - 1) > 0.01 || !origin.allFinite()){
          goon = true;
          continue;
        }
        marker.header.seq++;
        marker.id++;
        marker.pose.orientation = utils::trans::quat2quat(utils::trans::dcm2quat(triad));
        marker.pose.position.x = origin(0);
        marker.pose.position.y = origin(1);
        marker.pose.position.z = origin(2);
        // marker.colors.push_back(red);
        markers.markers.push_back(marker);
        goon = true;
      }
    }
    planes_publ.publish(markers);
  }

  if(plot_segments){
    ColoredPointCloud colored_segments;
    PointCloud pc;
    ColoredPointCloud color_pc;
    Eigen::Vector3d color;
    uint32_t rgb;
    int r, g, b;

    bool goon = true;
    for(int seg = 0 ; goon ; seg++){
      goon = false;
      if(pc2surfs.get_segment_pc(seg, pc)){
        pcl::copyPointCloud(pc, color_pc);
        color = colors[2 * (seg + 2)] * 255;
        r = color(0); g = color(1); b = color(2);

        //cout << "PC Color[" << seg << "] = [" << color(0) << ", " << color(1) << ", " << color(2) << "]" << endl;

        rgb = (static_cast<uint32_t>(r) << 16 |
            static_cast<uint32_t>(g) << 8  |
            static_cast<uint32_t>(b));
        for(auto &pt : color_pc.points)
          pt.rgb = *reinterpret_cast<float*>(&rgb);

        colored_segments += color_pc;
        goon = true;
      }

      if(seg != 0 && pc2surfs.get_segment_pc(-seg, pc)){
        pcl::copyPointCloud(pc, color_pc);
        color = colors[2 * (seg + 2) + 1] * 255;
        r = color(0); g = color(1); b = color(2);

        //cout << "PC Color[" << -seg << "] = [" << color(0) << ", " << color(1) << ", " << color(2) << "]" << endl;

        rgb = (static_cast<uint32_t>(r) << 16 |
            static_cast<uint32_t>(g) << 8  |
            static_cast<uint32_t>(b));
        for(auto &pt : color_pc.points)
          pt.rgb = *reinterpret_cast<float*>(&rgb);

        colored_segments += color_pc;
        goon = true;
      }
    }

    colored_segments.header.frame_id = "velodyne";
    colored_segments.height = 1;
    colored_segments.width = colored_segments.points.size();
    colored_segments.header.stamp = ros::Time::now().toSec();

    segments_publ.publish(colored_segments);
  }

  if(plot_cylinders){
    markers.markers.clear();
    marker.colors.clear();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.z = params.segment_len;

    Eigen::Vector3d origin;
    Eigen::Matrix3d triad;
    Eigen::MatrixXd contour;
    Eigen::Vector3d color;
    marker.points.clear();
    marker.colors.clear();
    marker.color.a = cylinders_alpha;
    marker.scale.x = marker.scale.y = marker.scale.z = cylinders_line_scale;
    double R;

    vector<Eigen::Vector3d> cylinder_pts;
    cylinder_pts.reserve(cylinders_resolution);
    for(int i = 0 ; i <= cylinders_resolution ; i++){
      double theta = 2.0 * i / cylinders_resolution * M_PI;
      Eigen::Vector3d pt(-params.segment_len / 2, cos(theta), sin(theta));
      cylinder_pts.push_back(pt);
    }

    bool goon = true;
    for(int seg = 0 ; goon ; seg++){
      goon = false;
      auto it = segment_origins.find(seg);
      if(it != segment_origins.end()){
        origin = it->second;
        triad = segment_triads[seg];
        R = segment_contours[seg](2);
        //cout << "R = " << R << endl;
        //cout << "det = " << triad.determinant() << endl;
        //cout << "seg = " << seg << endl;
        if(!triad.allFinite() || fabs(triad.determinant() - 1) > 0.01 || !origin.allFinite() || !isfinite(R)){
          goon = true;
        } else {
          marker.header.seq++;
          marker.id++;
          marker.pose.orientation = utils::trans::quat2quat(utils::trans::dcm2quat(triad));
          marker.pose.position.x = origin(0);
          marker.pose.position.y = origin(1);
          marker.pose.position.z = origin(2);
          color = colors[2 * (seg + 2)];
          marker.color.r = color(0);
          marker.color.g = color(1);
          marker.color.b = color(2);

          //cout << "Cyl. Color[" << seg << "] = [" << color(0) << ", " << color(1) << ", " << color(2) << "]" << endl;

          marker.points.clear();
          for(int i = 0 ; i < cylinders_resolution ; i++){
            geometry_msgs::Point pt1, pt2;
            pt1.x = cylinder_pts[i](0);
            pt1.y = cylinder_pts[i](1) * R;
            pt1.z = cylinder_pts[i](2) * R;
            pt2.x = cylinder_pts[i + 1](0);
            pt2.y = cylinder_pts[i + 1](1) * R;
            pt2.z = cylinder_pts[i + 1](2) * R;

            marker.points.push_back(pt1);
            marker.points.push_back(pt2);

            pt2.x += params.segment_len;
            marker.points.push_back(pt1);
            marker.points.push_back(pt2);

            marker.points.push_back(pt1);
            pt1.x += params.segment_len;
            marker.points.push_back(pt1);

            marker.points.push_back(pt1);
            marker.points.push_back(pt2);
          }

          markers.markers.push_back(marker);
          goon = true;
        }
      }

      it = segment_origins.find(-seg);
      if(seg != 0 && it != segment_origins.end()){
        origin = it->second;
        triad = segment_triads[-seg];
        //cout << "R = " << R << endl;
        //cout << "det = " << triad.determinant() << endl;
        //cout << "seg = " << seg << endl;
        if(!triad.allFinite() || fabs(triad.determinant() - 1) > 0.01 || !origin.allFinite() || !isfinite(R)){
          goon = true;
        } else {
          marker.header.seq++;
          marker.id++;
          marker.pose.orientation = utils::trans::quat2quat(utils::trans::dcm2quat(triad));
          marker.pose.position.x = origin(0);
          marker.pose.position.y = origin(1);
          marker.pose.position.z = origin(2);
          color = colors[2 * (seg + 2) + 1];
          marker.color.r = color(0);
          marker.color.g = color(1);
          marker.color.b = color(2);

          //cout << "Cyl. Color[" << -seg << "] = [" << color(0) << ", " << color(1) << ", " << color(2) << "]" << endl;

          marker.points.clear();
          for(int i = 0 ; i < cylinders_resolution ; i++){
            geometry_msgs::Point pt1, pt2;
            pt1.x = cylinder_pts[i](0);
            pt1.y = cylinder_pts[i](1) * R;
            pt1.z = cylinder_pts[i](2) * R;
            pt2.x = cylinder_pts[i + 1](0);
            pt2.y = cylinder_pts[i + 1](1) * R;
            pt2.z = cylinder_pts[i + 1](2) * R;

            marker.points.push_back(pt1);
            marker.points.push_back(pt2);

            pt2.x += params.segment_len;
            marker.points.push_back(pt1);
            marker.points.push_back(pt2);

            marker.points.push_back(pt1);
            pt1.x += params.segment_len;
            marker.points.push_back(pt1);

            marker.points.push_back(pt1);
            marker.points.push_back(pt2);
          }

          markers.markers.push_back(marker);
          goon = true;
        }
      }
    }
    cylinders_publ.publish(markers);
  }

  if(plot_sensor_frame){
    marker.points.clear();
    marker.colors.clear();
    marker.points.reserve(6);
    marker.colors.reserve(6);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.header.seq++;
    marker.id++;
    marker.scale.x = 3 * triads_scale;
    marker.scale.y = 3 * triads_scale;
    marker.scale.z = 3 * triads_scale;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;

    std_msgs::ColorRGBA red, green, blue;
    red.r = 1; 
    green.g = 1;
    blue.b = 1;
    red.a = green.a = blue.a = 1;

    Eigen::Vector3d origin(0, 0, 0);
    Eigen::Matrix3d triad = Eigen::Matrix3d::Identity();
    geometry_msgs::Point pt1, pt2;

    pt1.x = origin(0); pt1.y = origin(1); pt1.z = origin(2);
    pt2.x = pt1.x + triad(0, 0); pt2.y = pt1.y + triad(1, 0); pt2.z = pt1.z + triad(2, 0);
    marker.points.push_back(pt1); marker.points.push_back(pt2);
    marker.colors.push_back(red);
    marker.colors.push_back(red);
    pt2.x = pt1.x + triad(0, 1); pt2.y = pt1.y + triad(1, 1); pt2.z = pt1.z + triad(2, 1);
    marker.points.push_back(pt1); marker.points.push_back(pt2);
    marker.colors.push_back(green);
    marker.colors.push_back(green);
    pt2.x = pt1.x + triad(0, 2); pt2.y = pt1.y + triad(1, 2); pt2.z = pt1.z + triad(2, 2);
    marker.points.push_back(pt1); marker.points.push_back(pt2);
    marker.colors.push_back(blue);
    marker.colors.push_back(blue);

    sensor_frame_publ.publish(marker);
  }

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
  //
  seq = marker.header.seq;
  marker_id = marker.id;
}

void publish_odom_msg(){

  static int seq = 0;

  Eigen::Matrix3d imu_dcm = utils::trans::imu2dcm(imu_msg, true);
  Eigen::Vector3d imu_rpy = utils::trans::dcm2rpy(imu_dcm);
 
  if(imu_rpy(0) == 0 || imu_rpy(1) == 0){
    ROS_WARN("No measurement available! IMU roll and/or pitch are '0'.");
    return;
  }


  std::map<int, Eigen::Vector3d> segment_origins;
  std::map<int, Eigen::Matrix3d> segment_triads;
  std::map<int, Eigen::VectorXd> segment_contours;
  pc2surfs.get_segments(segment_origins, segment_triads, segment_contours);
   
  if(segment_origins.find(0) == segment_origins.end() ||
     segment_triads.find(0)  == segment_triads.end()){
    ROS_WARN("No measurement available! No segment at '0'.");
    return;
  }

  Eigen::Matrix4d se3;
  se3.setIdentity();
  se3.topLeftCorner<3, 3>() = segment_triads[0];
  se3.topRightCorner<3, 1>() = segment_origins[0];

  se3 = se3.inverse();
  
  se3(0, 3) = 0;

  Eigen::Vector3d rpy = utils::trans::dcm2rpy(se3.topLeftCorner<3, 3>());
  rpy(0) = imu_rpy(0);
  rpy(1) = imu_rpy(1);
  se3.topLeftCorner<3, 3>() = utils::trans::rpy2dcm(rpy);

  odom_msg = utils::trans::se32odom(se3);
  
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.seq = seq++;
  odom_msg.header.frame_id = "world";

  odom_publ.publish(odom_msg);

  static tf::TransformBroadcaster tf_br;
  tf::Transform tf_velodyne_world;
  tf_velodyne_world.setOrigin(tf::Vector3(se3(0, 3), se3(1, 3), se3(2, 3)));
  tf::Quaternion q;
  q.setRPY(rpy(0), rpy(1), rpy(2));
  tf_velodyne_world.setRotation(q);
  tf_br.sendTransform(tf::StampedTransform(tf_velodyne_world, ros::Time::now(), "world", "velodyne"));

  imu_available = false;
}


