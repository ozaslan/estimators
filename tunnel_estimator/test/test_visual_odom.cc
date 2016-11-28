#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <camera_calib_params.hh>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "utils.hh"
#include "pc_to_surfaces.hh"
#include "visual_1d_odom.hh"

double x = 0;

ros::Subscriber velodyne_subs;
ros::Subscriber cam_subs;
sensor_msgs::PointCloud2 velodyne_msg;
VisionBasedTunnelLocalizer vbtl;

CameraCalibParams top_left_cam_calib_params;
CameraCalibParams top_right_cam_calib_params;
CameraCalibParams bottom_right_cam_calib_params;
CameraCalibParams bottom_left_cam_calib_params;

cv_bridge::CvImagePtr image_msg;

pcl::PointCloud<pcl::PointXYZ>::Ptr local_map = NULL;

string top_left_cam_calib_file,
       top_right_cam_calib_file,
       bottom_right_cam_calib_file,
       bottom_left_cam_calib_file;

void process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
void velodyne_callback(const sensor_msgs::PointCloud2 &msg);
void cam_callback(const sensor_msgs::Image &msg);

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

  n.param("top_left_cam_calib_file"		 , top_left_cam_calib_file	  , string("ERROR"));
  n.param("top_right_cam_calib_file"	 , top_right_cam_calib_file	  , string("ERROR"));
  n.param("bottom_right_cam_calib_file", bottom_right_cam_calib_file, string("ERROR"));
  n.param("bottom_left_cam_calib_file" , bottom_left_cam_calib_file  , string("ERROR"));

  n.param("debug_mode", debug_mode, false);



  ROS_INFO(" ---------- TEST_PC2SURFACES NODE ------------");
  ROS_INFO("[debug_mode] : [%s]", debug_mode ? "TRUE" : "FALSE");
  ROS_INFO("[top_left_cam_calib_file] ----- : [%s]", top_left_cam_calib_file.c_str());
  ROS_INFO("[top_right_cam_calib_file] ---- : [%s]", top_right_cam_calib_file.c_str());
  ROS_INFO("[bottom_right_cam_calib_file] - : [%s]", bottom_right_cam_calib_file.c_str());
  ROS_INFO("[bottom_left_cam_calib_file] -- : [%s]", bottom_left_cam_calib_file.c_str());
  params.print();
  ROS_INFO(" ------------------------------------------");

  top_left_cam_calib_params.load(top_left_cam_calib_file);
  ROS_INFO("Top Left Cam Calib Params :");
  top_left_cam_calib_params.print();

  /*
     top_right_cam_calib_params.load(top_right_cam_calib_file);
     ROS_INFO("Top Right Cam Calib Params :");
     top_right_cam_calib_params.print();

     bottom_left_cam_calib_params.load(bottom_left_cam_calib_file);
     ROS_INFO("Bottom Left Cam Calib Params :");
     bottom_left_cam_calib_params.print();

     bottom_left_cam_calib_params.load(bottom_left_cam_calib_file);
     ROS_INFO("Bottom Left Cam Calib Params :");
     bottom_left_cam_calib_params.print();
     */
  pc2surfs.set_params(params);

  vector<CameraCalibParams> camera_calib_params;
  camera_calib_params.push_back(top_left_cam_calib_params);

  vbtl.register_camera_params(camera_calib_params);
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
    //ROS_INFO(" --- Publishing : ~pose_grapelodyne_subs	= n.subscribe("velodyne_points", 10, velodyne_callback, ros::TransportHints().tcpNoDelay());
  cam_subs	    = n.subscribe("cam", 3000, cam_callback, ros::TransportHints().tcpNoDelay());

  return 0;
}

void cam_callback(const sensor_msgs::Image &msg){
  if(debug_mode)
    ROS_INFO("TEST PC2SURFACES NODE : Got Camera message!");

  /*
  try	{
    image_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)	{
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  */

  vector<cv::Mat> frames;

  try	{
    cv::Mat color_img, gray_img;
    image_msg = cv_bridge::toCvCopy(msg);
    cv::cvtColor(image_msg->image, color_img, CV_BayerBG2BGR);

    char buffer [50];
    sprintf (buffer, "%05d", image_msg->header.seq);

    string filename = "/media/ozaslan/research_backup/temp/image_" + string(buffer) + ".png";
    cv::imwrite(filename, color_img);
    cout << "Writing image to <" << filename << "> " << endl;
    cout << "# channels : " << color_img.channels() << endl;
    cout << "Encoding : " << msg.encoding << endl;
    //cv::namedWindow("color");
    //cv::imshow("color", color_img);
    //cv::waitKey(1);
    return;
    vector<cv::Mat> channels;

    cv::split(color_img, channels); 
    cv::equalizeHist(channels[0], channels[0]);
    cv::equalizeHist(channels[1], channels[1]);
    cv::equalizeHist(channels[2], channels[2]);
    cv::merge(channels, color_img);

    
    cv::imshow("color", channels[0] - channels[1]);
    return;

    if(color_img.channels() == 3)
      cv::cvtColor(color_img, gray_img, CV_BGR2GRAY);
    else if(color_img.channels() == 4)
      cv::cvtColor(color_img, gray_img, CV_BGRA2GRAY);
    else
      gray_img = color_img;

    cv::GaussianBlur(gray_img, gray_img, cv::Size(13, 13), 5, 5, BORDER_DEFAULT );

    cv::Mat mask2;
    cv::threshold(gray_img, mask2, 150, 255, cv::THRESH_BINARY);

    cv::adaptiveThreshold(gray_img, gray_img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 55, -2.5);

    frames.push_back(gray_img);
  }
  catch (cv_bridge::Exception& e)	{
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  vbtl.push_camera_data(frames, Eigen::Matrix4d::Identity());
  double x_disp;
  vbtl.estimate_displacement(x_disp);
  x += x_disp;
  cout << "x position : " << x << endl;
  // vbtl.get_displacement(x_disp);
  // vbtl.get_variance(x_var);
  // bool TunnelLocalizer::get_back_projected_features(vector<pcl::PointXYZ> &tails, vector<pcl::PointXYZ> &tips){
  // vbtl.get_back_projected_flow_vectors(tails, tips);
  frames.clear();
  frames.push_back(image_msg->image);
  vbtl.plot_flows(frames, true, true);
  frames.resize(2);
  cv::undistort(frames[0], frames[1], top_left_cam_calib_params.camera_matrix, top_left_cam_calib_params.dist_coeffs);
  cv::namedWindow("OpticalFlow");
  cv::imshow("OpticalFlow", frames[1]);
  cv::waitKey(1);
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

  return;

  pc2surfs.get_segments(segment_origins, segment_triads, segment_contours);

  if(local_map == NULL)
    local_map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Vector3d &origin  = segment_origins[0];
  Eigen::Matrix3d &triad   = segment_triads[0];
  Eigen::VectorXd &contour = segment_contours[0];

  local_map->points.clear();
  local_map->points.reserve(4000);

  int num_pts = 100;
  double R  = contour(2);
  double yc = contour(0);
  double zc = contour(1);

  for(int i = 0 ; i < num_pts ; i++){
    Eigen::Vector3d pt;
    pt(1) = R * cos(i / (double)num_pts * 2 * M_PI) + yc;
    pt(2) = R * sin(i / (double)num_pts * 2 * M_PI) + zc;
    for(double x = -1 ; x <= 1 ; x += 0.05){
      pt(0) = x;
      pt = triad * pt + origin;
      local_map->points.push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
    }
  }

  /*
  pc2surfs.get_segment_pc(0, *local_map);

  cout << "local_map.points.size() = " << local_map->points.size() << endl;

  cout << "map_pts = [" << endl;
  for(int i = 0 ; i < local_map->points.size() ; i++){
    pcl::PointXYZ &pt = local_map->points[i];
    cout << pt.x << " " << pt.y << " " << pt.z << ";" << endl;
  }
  cout << "];" << endl;
  */
  vbtl.set_map(local_map);

}

