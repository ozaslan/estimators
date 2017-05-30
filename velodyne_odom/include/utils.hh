/*
   This header file defines a set of mathematics utility 
   functions. Additionally, other header files which define
   further utility functions such as representation transformations,
   lidar data processing. The user should include this header file
   to benefit from all the utiity function while keeping in mind
   the namespace structure.
 */

#ifndef PI
#define PI 3.14159265359
#endif

#include <cmath>
#include <limits>
#include <vector>
#include <cassert>
#include <iostream>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include "trans_utils.hh"

// #include <armadillo>

using namespace std;
using namespace Eigen;
//using namespace arma;

// Putting this header lock at the beginning of the file prevents
// including the library headers in the preamble of this file.
// However functions in the "trans_utils" and "laser_utils" headers
// utilizes these libraries.

#ifndef _UTILS_HH_
#define _UTILS_HH_

// Define new Eigen matrix and vector types
namespace Eigen{
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 5, 1> Vector5d;
}

namespace utils{
  // This macro provides an efficient way to raise exceptions with
  // very informative command line messages.
#define ASSERT(condition, message) \
  if (! (condition)) { \
    std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
    << " line " << __LINE__ << ": " << message << std::endl; \
    std::exit(EXIT_FAILURE); \
  } \

#define PRINT_FLF {cout << "File : " << __FILE__ << " Line # : " << __LINE__ << " Func : " << __func__ << endl;}

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) / 180.0 * PI)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) ((x) / PI * 180.0)
#endif

  // The 'utils' namespace implements utility functions which cannot
  // be categorized into a specific class of helper routines. However
  // functions such as rotation transformations and ROS-Eigen data
  // structure conversions are collected under the 'utils::trans'
  // namespace in its specific header and source files. Similarly
  // laser processing routines are grouped under 'utils::laser'
  // namespace. As new functions are required, they should be added
  // directly under 'utils' namespace unless they intuitively form
  // a group/set of functions.

  class Timer{
    private:
      ros::Time _start;
      bool _print_on;
      string _label;
    public:
      Timer(bool print_on = true, string label = ""){
        _print_on = print_on;
        _label    = label;
      }

      void tic(){
        _start = ros::Time::now();
      }

      void print_on() {_print_on = true ;}
      void print_off(){_print_on = false;}

      double toc(string sublabel = ""){
        double dt = (ros::Time::now() - _start).toSec();
        if(_print_on == true)
          cout << "<" << _label << sublabel << " Time elapsed : " << dt << ">" << endl;
        return dt;
      }
  };

  // Clamps the given value in between the extrema
  inline double clamp(double val, double min, double max){
    return val > max ? max : val < min ? min : val;
  }

  /*
     inline void clamp(double &val, double min, double max){
     val = val > max ? max : val < min ? min : val;
     }
   */

  // This function returns the 2*PI modula shifted by -PI
  // of an angle in radians
  inline double fix_angle(double ang){
    while(ang > PI)
      ang -= PI;
    while(ang <= -PI)
      ang += PI;
    return ang;
  }

  inline void fix_angle(double *ang){
    while(*ang > PI)
      *ang -= PI;
    while(*ang <= -PI)
      *ang += PI;
  }

  inline void generate_colors(vector<Eigen::Vector3i> &colors){
    static int temp_colors[] = {
      0x000000, 0x00FF00, 0x0000FF, 0xFF0000,
      0x01FFFE, 0xFFA6FE, 0xFFDB66, 0x006401,
      0x010067, 0x95003A, 0x007DB5, 0xFF00F6,
      0xFFEEE8, 0x774D00, 0x90FB92, 0x0076FF,
      0xD5FF00, 0xFF937E, 0x6A826C, 0xFF029D,
      0xFE8900, 0x7A4782, 0x7E2DD2, 0x85A900,
      0xFF0056, 0xA42400, 0x00AE7E, 0x683D3B,
      0xBDC6FF, 0x263400, 0xBDD393, 0x00B917,
      0x9E008E, 0x001544, 0xC28C9F, 0xFF74A3,
      0x01D0FF, 0x004754, 0xE56FFE, 0x788231,
      0x0E4CA1, 0x91D0CB, 0xBE9970, 0x968AE8,
      0xBB8800, 0x43002C, 0xDEFF74, 0x00FFC6,
      0xFFE502, 0x620E00, 0x008F9C, 0x98FF52,
      0x7544B1, 0xB500FF, 0x00FF78, 0xFF6E41,
      0x005F39, 0x6B6882, 0x5FAD4E, 0xA75740,
      0xA5FFD2, 0xFFB167, 0x009BFF, 0xE85EBE};
    colors.clear();
    colors.resize(64);
    for(int i = 0 ; i < 64 ; i++){
      colors[i](0) = (temp_colors[i] & 0xFF0000) >> 16;
      colors[i](1) = (temp_colors[i] & 0x00FF00) >> 8;
      colors[i](2) = (temp_colors[i] & 0x0000FF);
    }
  }  
/*
  typedef struct Ellipse
  {
    arma::vec::fixed<2> center;
    arma::vec::fixed<2> major_axis;
    arma::vec::fixed<2> minor_axis;
    float len_major, len_minor;
    float theta;
    bool success;
  } Ellipse;

  // Output is A = [a b c d e f]' such that 'ax^2 + bxy + cy^2 +dx + ey + f = 0'
  bool fit_ellipse_ransac(vector<double> &xs, vector<double> &ys, vector<double> &fit, int num_trials = 200);
  void fit_ellipse(vector<double> &xs, vector<double> &ys, vector<double> &fit);
  void get_ellipse_parameters(vector<double> &fit, Ellipse &ell);
*/
}

#endif

